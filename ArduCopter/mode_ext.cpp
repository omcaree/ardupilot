//#include "mode.h"
#include "Copter.h"
#include <AP_AHRS/AP_AHRS.h>

bool ModeExt::init(bool ignore_checks) {

    const AP_SerialManager& serial_manager = AP::serialmanager();

    // locate communication serial port (configured via ardupilot parameters)
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ExtMode, 0);
    if (_uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No External Mode Serial Port!");
        return false;
    }

    // disable flow control
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    // default outputs to zero
    _zero_output();

    // reset state
    _reset_ext();

    // start first control clock cycle
    _cycle_start_time = AP_HAL::micros();

    // send output, if sending fails, fail the init
    if (!_send_data_to_ext()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "External Mode Init Failed!");
        return false;
    }

    _companion_state = ExtState::WAIT_FOR_RESPONSE;
    gcs().send_text(MAV_SEVERITY_INFO,"External Mode Enabled");
    if (copter.g2.ext_unsafe>0) {
        gcs().send_text(MAV_SEVERITY_WARNING,"External Mode Unsafe!");
    }
    return true;
}

void ModeExt::run() {
    // check if compute board has responded
    uint32_t cycle_duration = AP_HAL::micros() - _cycle_start_time;

    // Check if the response from companion computer is taking too long
    if (cycle_duration >= TIMEOUT) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "External Mode Timeout (%ld %ld)!", AP_HAL::micros(), _cycle_start_time);
        if (copter.g2.ext_unsafe==0) {
            _set_mode_stabilize();
            return;
        }
    }

    // waiting for response from the companion computer
    if (_companion_state == ExtState::WAIT_FOR_RESPONSE && _data_available()) {
        _receive_from_ext();
        output_to_motors(); 
        _companion_state = ExtState::READY_FOR_REQUEST;
    }

    // ready for next control cycle?
    if (_companion_state == ExtState::READY_FOR_REQUEST) {
        // is it time for next clock cycle (100Hz timing check)
        // as both micros and _cycle_time are uint32, this code handles overflow correctly
        if (cycle_duration >= CLOCK_PERIOD) {

            // calculate the new start time to prevent accumulation of small execution delays
            // Note: if the end of the next cycle has already been exceeded then run() has
            // not been called frequently enough to keep ahead of the controller timing
            // deadlines. The drone is likely to be behaving erratically if this has occurred.
            _cycle_start_time += CLOCK_PERIOD;
            if (AP_HAL::micros() - _cycle_start_time >= CLOCK_PERIOD) {
                _cycle_start_time = AP_HAL::micros();
            }
            // Send data to companion computer. If sending fails, exit
            if (!_send_data_to_ext()) {
                return;
            }
            _companion_state = ExtState::WAIT_FOR_RESPONSE;
        }
    }
}

// Send outputs to servos channels
void ModeExt::output_to_motors() {
    float throttle = _outputs[Outputs::THROTTLE];
    uint16_t port_ail = (uint16_t)(1500 + (_outputs[Outputs::PORT_AILERON]*500));
    uint16_t stbd_ail = (uint16_t)(1500 + (_outputs[Outputs::STBD_AILERON]*500));
    uint16_t ele =(uint16_t)(1500 + ( _outputs[Outputs::ELEVATOR]*500));
    float port_col = _outputs[Outputs::PORT_COLLECTIVE];
    float port_lat = _outputs[Outputs::PORT_LATERAL_CYCLIC];
    float port_lon = _outputs[Outputs::PORT_LONGITUDINAL_CYCLIC];
    uint16_t port_tilt = (uint16_t)(1500 + (_outputs[Outputs::PORT_TILT]*500));
    float stbd_col = _outputs[Outputs::STBD_COLLECTIVE];
    float stbd_lat = _outputs[Outputs::STBD_LATERAL_CYCLIC];
    float stbd_lon = _outputs[Outputs::STBD_LONGITUDINAL_CYCLIC];
    uint16_t stbd_tilt = (uint16_t)(1500 + (_outputs[Outputs::STBD_TILT]*500));
    ext_motors->output_external(throttle, port_col, port_lat, port_lon, stbd_col, stbd_lat, stbd_lon);
    hal.rcout->write(port_ail_ch, port_ail);
    hal.rcout->write(stbd_ail_ch, stbd_ail);
    hal.rcout->write(ele_ch, ele);
    hal.rcout->write(port_tilt_ch, port_tilt);
    hal.rcout->write(stbd_tilt_ch, stbd_tilt);
}

// sends reset command to companion computer
void ModeExt::_reset_ext() const {
    _send_byte('r');
}

/*
 * Cleans UART receive buffer, reads attitude information from Ardupilot subsystems
 * and sends the data to the companion computer
 *
 * Returns false and revert to stablize mode if reading of data fails unless the "unsafe" parameter is set
 * 
 */
bool ModeExt::_send_data_to_ext() {
    // ensure the receive buffer is empty (in case we have a partial packet left over due to a communication issue)
    _uart->discard_input();

    // linear velocity in world frame
    Vector3f velocity;
    if (!ahrs.get_velocity_NED(velocity) & (copter.g2.ext_unsafe==0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No velocity. Reverting to Stabilize.");
        _set_mode_stabilize();
        return false;
    }

    // linear acceleration in vehicle frame
    Vector3f acceleration = ahrs.get_accel() - ahrs.get_accel_bias();

    // angular velocity in vehicle frame
    Vector3f gyro = ahrs.get_gyro();

    // quaternion 
    Quaternion quaternion;
    if (!ahrs.get_quaternion(quaternion) & (copter.g2.ext_unsafe==0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No quaternion. Reverting to Stabilize.");
        _set_mode_stabilize();
        return false;
    }

    // pilot stick positions from RC
    float roll = copter.channel_roll->norm_input();
    float pitch = copter.channel_pitch->norm_input();
    float throttle = copter.channel_throttle->norm_input();

    // AOA and SSA
    float aoa = ToRad(ahrs.getAOA());
    float ssa = ToRad(ahrs.getSSA());

    // airspeed
    float airspeed;
    if (!ahrs.airspeed_estimate(airspeed) & (copter.g2.ext_unsafe==0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No airspeed. Reverting to Stabilize.");
        _set_mode_stabilize();
        return false;
    }

    // send to act command to companion computer
    _send_byte('a');
    _send_uint32(AP_HAL::micros());
    _send_vector(velocity);
    _send_vector(acceleration);
    _send_quaternion(quaternion);
    _send_vector(gyro);
    _send_float(airspeed);
    _send_float(aoa);
    _send_float(ssa);
    _send_float(roll);
    _send_float(pitch);
    _send_float(throttle);

    // ensure data is sent immediately
    _uart->flush();
    return true;
}

bool ModeExt::_data_available() const {
    uint16_t available =  _uart->available();
    return available >= (12 * sizeof(uint16_t) + 1);
}

void ModeExt::_receive_from_ext() {
    // skip guard byte
    // There appears to be an undiagnosed electrical issue (missing pull up resistor to maintain idle state?)
    // that only appears to affect the first bit of the received data packet. For now, we are
    // inserting a guard byte to soak up the error.
    // This might be the cause: https://discuss.cubepilot.org/t/jetson-uart-connection-to-telem1-or-telem2/6258?replies_to_post_number=14
    _receive_uint8();

    _outputs[Outputs::THROTTLE] = _receive_float();
    _outputs[Outputs::PORT_AILERON] = _receive_float();
    _outputs[Outputs::STBD_AILERON] = _receive_float();
    _outputs[Outputs::ELEVATOR] = _receive_float();
    _outputs[Outputs::PORT_COLLECTIVE] = _receive_float();
    _outputs[Outputs::PORT_LATERAL_CYCLIC] = _receive_float();
    _outputs[Outputs::PORT_LONGITUDINAL_CYCLIC] = _receive_float();
    _outputs[Outputs::PORT_TILT] = _receive_float();
    _outputs[Outputs::STBD_COLLECTIVE] = _receive_float();
    _outputs[Outputs::STBD_LATERAL_CYCLIC] = _receive_float();
    _outputs[Outputs::STBD_LONGITUDINAL_CYCLIC] = _receive_float();
    _outputs[Outputs::STBD_TILT] = _receive_float();
}

void ModeExt::_send_byte(const uint8_t v) const {
    _uart->write(v);
}

void ModeExt::_send_uint32(const uint32_t v) const {
    _uart->write((uint8_t *) &v, sizeof(v));
}

void ModeExt::_send_float(const float v) const {
    _uart->write((uint8_t *) &v, sizeof(v));
}

void ModeExt::_send_vector(const Vector3f v) const {
    _send_float(v.x);
    _send_float(v.y);
    _send_float(v.z);
}

void ModeExt::_send_quaternion(const Quaternion q) const {
    _send_float(q.q1);
    _send_float(q.q2);
    _send_float(q.q3);
    _send_float(q.q4);
}

uint8_t ModeExt::_receive_uint8() const {
    uint8_t v;
    _uart->read(&v, sizeof(v));
    return v;
}

uint16_t ModeExt::_receive_uint16() const {
    uint16_t v;
    _uart->read((uint8_t *) &v, sizeof(v));
    return v;
}

float ModeExt::_receive_float() const {
    float v;
    _uart->read((uint8_t *) &v, sizeof(v));
    return v;
}

void ModeExt::_zero_output() {
    _outputs[Outputs::THROTTLE] = -1.0f;
    _outputs[Outputs::PORT_AILERON] = 0.0f;
    _outputs[Outputs::STBD_AILERON] = 0.0f;
    _outputs[Outputs::ELEVATOR] = 0.0f;
    _outputs[Outputs::PORT_COLLECTIVE] = 0.0f;
    _outputs[Outputs::PORT_LATERAL_CYCLIC] = 0.0f;
    _outputs[Outputs::PORT_LONGITUDINAL_CYCLIC] = 0.0f;
    _outputs[Outputs::PORT_TILT] = 0.0f;
    _outputs[Outputs::STBD_COLLECTIVE] = 0.0f;
    _outputs[Outputs::STBD_LATERAL_CYCLIC] = 0.0f;
    _outputs[Outputs::STBD_LONGITUDINAL_CYCLIC] = 0.0f;
    _outputs[Outputs::STBD_TILT] = 0.0f;
}

void ModeExt::_set_mode_stabilize() {
    // Force mode back to something sensible to prevent craft locking up.
    if (!copter.set_mode(copter.mode_stabilize.mode_number(), TIMEOUT_MODE_REASON)) {
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Failed to switch to Stabilize. Setting zero thrust.");

        // Failed to change mode - let's cut the motors instead.
        _zero_output();
    }
}