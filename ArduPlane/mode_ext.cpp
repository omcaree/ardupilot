#include "mode.h"
#include "Plane.h"
#include <AP_AHRS/AP_AHRS.h>

bool ModeExt::_enter() {

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
    if (plane.g2.ext_soton>0) {
        gcs().send_text(MAV_SEVERITY_INFO,"Southampton Format");
    }
    if (plane.g2.ext_unsafe>0) {
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
        if (plane.g2.ext_unsafe==0) {
            _set_mode_stabilize();
            return;
        }
    }

    // waiting for response from the companion computer
    if (_companion_state == ExtState::WAIT_FOR_RESPONSE && _data_available()) {
        _receive_from_ext();
        _output(); 
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
void ModeExt::_output() {
    SRV_Channels::set_output_norm(SRV_Channel::k_throttle, _outputs[Outputs::THROTTLE]);
    SRV_Channels::set_output_norm(SRV_Channel::k_flaperon_left, _outputs[Outputs::PORT_AILERON]);
    SRV_Channels::set_output_norm(SRV_Channel::k_flaperon_right, _outputs[Outputs::STBD_AILERON]);
    SRV_Channels::set_output_norm(SRV_Channel::k_elevator, _outputs[Outputs::ELEVATOR]);
    if (plane.g2.ext_soton>0) {
        SRV_Channels::set_output_norm(SRV_Channel::k_rudder, _outputs[Outputs::RUDDER]);
    }
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
    if (!ahrs.get_velocity_NED(velocity) & (plane.g2.ext_unsafe==0)) {
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
    if (!ahrs.get_quaternion(quaternion) & (plane.g2.ext_unsafe==0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No quaternion. Reverting to Stabilize.");
        _set_mode_stabilize();
        return false;
    }

    // pilot stick positions from RC
    float roll = plane.channel_roll->norm_input();
    float pitch = plane.channel_pitch->norm_input();
    float throttle = plane.channel_throttle->norm_input();
    float rudder = plane.channel_rudder->norm_input();

    // AOA and SSA
    float aoa = ToRad(ahrs.getAOA());
    float ssa = ToRad(ahrs.getSSA());

    // airspeed
    float airspeed;
    if (!ahrs.airspeed_estimate(airspeed) & (plane.g2.ext_unsafe==0)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "No airspeed. Reverting to Stabilize.");
        _set_mode_stabilize();
        return false;
    }

    // position and body velocity
    Vector3f position;
    Vector3f body_velocity;
    if (plane.g2.ext_soton>0) {
        if (!ahrs.get_relative_position_NED_home(position) & (plane.g2.ext_unsafe==0)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "No position. Reverting to Stabilize.");
            _set_mode_stabilize();
            return false;
        }
        if (!ahrs.airspeed_vector_true(body_velocity) & (plane.g2.ext_unsafe==0)) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "No body velocity. Reverting to Stabilize.");
            _set_mode_stabilize();
            return false;
        }
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
    if (plane.g2.ext_soton>0) {
        _send_float(rudder);
        _send_vector(position);
        _send_vector(body_velocity);
    }

    // ensure data is sent immediately
    _uart->flush();
    return true;
}

bool ModeExt::_data_available() const {
    uint16_t available =  _uart->available();
    if (plane.g2.ext_soton>0) {
        return available >= (5 * sizeof(uint16_t) + 1);
    } else {
        return available >= (4 * sizeof(uint16_t) + 1);
    }
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
    if (plane.g2.ext_soton>0) {
        _outputs[Outputs::RUDDER] = _receive_float();
    }
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
}

void ModeExt::_set_mode_stabilize() {
    // Force mode back to something sensible to prevent craft locking up.
    if (!plane.set_mode(plane.mode_stabilize, TIMEOUT_MODE_REASON)) {
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Failed to switch to Stabilize. Setting zero thrust.");

        // Failed to change mode - let's cut the motors instead.
        _zero_output();
    }
}
