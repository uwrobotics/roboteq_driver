#include "roboteq_controller.hpp"

#include <memory>

#include "i_comm.hpp"

namespace roboteq {

roboteq_controller::roboteq_controller(std::unique_ptr<i_comm> &&comm) : comm_interface_(std::move(comm)) {}

// set methods
bool roboteq_controller::setMotorCommand(int32_t command, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_MOTOR_COMMAND, channel, command);
}

bool roboteq_controller::setPosition(int32_t position, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_POSITION, channel, position);
}

bool roboteq_controller::setVelocity(int32_t speed, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_VELOCITY, channel, speed);
}

bool roboteq_controller::setEncoderCounter(int32_t counter, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_ENCODER_COUNTER, channel, counter);
}

bool roboteq_controller::setBrushlessCounter(int32_t counter, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_BRUSHLESS_COUNTER, channel, counter);
}

bool roboteq_controller::setUserIntVariable(int32_t var, uint8_t nbvar) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_USER_INT_VARIABLE, nbvar, var);
}

bool roboteq_controller::setAcceleration(int32_t accel, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_ACCELERATION, channel, accel);
}

bool roboteq_controller::setDeceleration(int32_t decel, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_DECELERATION, channel, decel);
}

bool roboteq_controller::setAllDigitalOutBits(uint8_t out_bits) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_ALL_DIGITAL_OUT_BITS, 0, out_bits);
}

bool roboteq_controller::setIndividualDigitalOutBits(uint8_t out_bits) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0, out_bits);
}

bool roboteq_controller::resetIndividualOutBits(uint8_t out_bits) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::RESET_INDIVIDUAL_OUT_BITS, 0, out_bits);
}

bool roboteq_controller::loadHomeCounter(uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::LOAD_HOME_COUNTER, 0, channel);
}

bool roboteq_controller::emergencyShutdown() {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::EMERGENCY_SHUTDOWN, 0);
}

bool roboteq_controller::releaseShutdown() {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::RELEASE_SHUTDOWN, 0, 0);
}

bool roboteq_controller::stopInAllModes(uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::STOP_IN_ALL_MODES, channel, 0);
}

bool roboteq_controller::setPosRelative(int32_t position, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_POS_RELATIVE, channel, position);
}

bool roboteq_controller::setNextPosAbsolute(int32_t position, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_NEXT_POS_ABSOLUTE, channel, position);
}

bool roboteq_controller::setNextPosRelative(int32_t position, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_NEXT_POS_RELATIVE, channel, position);
}

bool roboteq_controller::setNextAcceleration(int32_t accel, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_NEXT_ACCELERATION, channel, accel);
}

bool roboteq_controller::setNextDeceleration(int32_t decel, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_NEXT_DECELERATION, channel, decel);
}

bool roboteq_controller::setNextVelocity(int32_t speed, uint8_t channel) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_NEXT_VELOCITY, channel, speed);
}

bool roboteq_controller::setUserBoolVariable(uint32_t var, uint8_t nbvar) {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SET_USER_BOOL_VARIABLE, nbvar, var);
}

bool roboteq_controller::saveConfigToFlash() {
  return comm_interface_->sdo_download(roboteq::send_runtime_command::SAVE_CONFIG_TO_FLASH, 0, 0);
}

// read methods
int16_t roboteq_controller::readMotorAmps(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel)) *
         MOTOR_AMPS_READING_CONVERSION_FACTOR;
}

int16_t roboteq_controller::readActualMotorCommand(uint8_t channel) {
  return static_cast<int16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ACTUAL_MOTOR_COMMAND, channel));
}

int16_t roboteq_controller::readAppliedPowerLevel(uint8_t channel) {
  return static_cast<int16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ACTUAL_POWER_LEVEL, channel));
}

int32_t roboteq_controller::readEncoderMotorSpeed(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_MOTOR_SPEED, channel));
}

int32_t roboteq_controller::readAbsoluteEncoderCount(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ABSOLUTE_ENCODER_COUNT, channel));
}

int32_t roboteq_controller::readAbsoluteBrushlessCounter(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ABSOLUTE_BRUSHLESS_COUNTER, channel));
}

int32_t roboteq_controller::readUserIntegerVariable(int32_t nbvar) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_USER_INTEGER_VARIABLE, nbvar));
}

int16_t roboteq_controller::readEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdo_upload(
      roboteq::send_runtime_query::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
}

int32_t roboteq_controller::readEncoderCountRelative(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_COUNT_RELATIVE, channel));
}

int32_t roboteq_controller::readBrushlessCountRelative(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_COUNT_RELATIVE, channel));
}

int16_t roboteq_controller::readBrushlessMotorSpeed(uint8_t channel) {
  return static_cast<int16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED, channel));
}

int16_t roboteq_controller::readBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdo_upload(
      roboteq::send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
}

int16_t roboteq_controller::readBatteryAmps(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_BATTERY_AMPS, channel));
}

uint16_t roboteq_controller::readInternalVoltages(uint8_t param) {
  return static_cast<uint16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_VOLTAGES, param));
}

uint32_t roboteq_controller::readAllDigitalInputs() {
  return static_cast<uint32_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_ALL_DIGITAL_INPUTS));
}

int8_t roboteq_controller::readCaseAndInternalTemperatures(uint8_t param) {
  return static_cast<int8_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_CASE_AND_INTERNAL_TEMPERATURES, param));
}

int16_t roboteq_controller::readFeedback(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_FEEDBACK, channel));
}

uint16_t roboteq_controller::readStatusFlags() {
  return static_cast<uint16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_STATUS_FLAGS));
}

uint16_t roboteq_controller::readFaultFlags() {
  return static_cast<uint16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_FAULT_FLAGS));
}

uint16_t roboteq_controller::readCurrentDigitalOutputs() {
  return static_cast<uint16_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_CURRENT_DIGITAL_OUTPUTS));
}

int32_t roboteq_controller::readClosedLoopError(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_CLOSED_LOOP_ERROR, channel));
}

bool roboteq_controller::readUserBooleanVariable(uint32_t nbvar) {
  return static_cast<bool>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_USER_BOOLEAN_VARIABLE, nbvar));
}

int32_t roboteq_controller::readInternalSerialCommand(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_SERIAL_COMMAND, channel));
}

int32_t roboteq_controller::readInternalAnalogCommand(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_ANALOG_COMMAND, channel));
}

int32_t roboteq_controller::readInternalInternalPulseCommand(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_PULSE_COMMAND, channel));
}

uint32_t roboteq_controller::readTime(uint8_t param) {
  return static_cast<uint32_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_TIME, param));
}

uint16_t roboteq_controller::readSpektrumRadioCapture(uint8_t nb_capture) {
  return static_cast<uint16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_SPEKTRUM_RADIO_CAPTURE, nb_capture));
}

uint8_t roboteq_controller::readDestinationPositionReachedFlag(uint8_t channel) {
  return static_cast<uint8_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_DESTINATION_POSITION_REACHED_FLAG, channel));
}

int32_t roboteq_controller::readMEMSAccelerometerAxis(uint8_t axis) {
  return static_cast<int32_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MEMS_ACCELEROMETER_AXIS, axis));
}

uint8_t roboteq_controller::readMagsensorTrackDetect() {
  return static_cast<uint8_t>(comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_TRACK_DETECT));
}

int16_t roboteq_controller::readMagsensorTrackPosition(uint8_t nb_pulse) {
  return static_cast<int16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_TRACK_POSITION, nb_pulse));
}

uint8_t roboteq_controller::readMagsensorMarkers(uint8_t nb_pulse) {
  return static_cast<uint8_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_MARKERS, nb_pulse));
}

uint16_t roboteq_controller::readMagsensorStatus(uint8_t nb_pulse) {
  return static_cast<uint16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_STATUS, nb_pulse));
}

uint16_t roboteq_controller::readMotorStatusFlags(uint8_t nb_pulse) {
  return static_cast<uint16_t>(
      comm_interface_->sdo_upload(roboteq::send_runtime_query::READ_MOTOR_STATUS_FLAGS, nb_pulse));
}

}  // namespace roboteq
