#include "RoboteqController.hpp"

#include <memory>

#include "CommunicationInterface.hpp"

namespace roboteq {

RoboteqController::RoboteqController(std::unique_ptr<CommunicationInterface> &&comm)
    : comm_interface_(std::move(comm)) {}

// set methods
bool RoboteqController::setMotorCommand(int32_t command, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_MOTOR_COMMAND, channel, command);
}

bool RoboteqController::setPosition(int32_t position, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_POSITION, channel, position);
}

bool RoboteqController::setVelocity(int32_t speed, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_VELOCITY, channel, speed);
}

bool RoboteqController::setEncoderCounter(int32_t counter, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_ENCODER_COUNTER, channel, counter);
}

bool RoboteqController::setBrushlessCounter(int32_t counter, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_BRUSHLESS_COUNTER, channel, counter);
}

bool RoboteqController::setUserIntVariable(int32_t var, uint8_t nbvar) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_USER_INT_VARIABLE, nbvar, var);
}

bool RoboteqController::setAcceleration(int32_t accel, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_ACCELERATION, channel, accel);
}

bool RoboteqController::setDeceleration(int32_t decel, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_DECELERATION, channel, decel);
}

bool RoboteqController::setAllDigitalOutBits(uint8_t out_bits) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_ALL_DIGITAL_OUT_BITS, 0, out_bits);
}

bool RoboteqController::setIndividualDigitalOutBits(uint8_t out_bits) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0, out_bits);
}

bool RoboteqController::resetIndividualOutBits(uint8_t out_bits) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::RESET_INDIVIDUAL_OUT_BITS, 0, out_bits);
}

bool RoboteqController::loadHomeCounter(uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::LOAD_HOME_COUNTER, 0, channel);
}

bool RoboteqController::emergencyShutdown() {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::EMERGENCY_SHUTDOWN, 0);
}

bool RoboteqController::releaseShutdown() {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::RELEASE_SHUTDOWN, 0, 0);
}

bool RoboteqController::stopInAllModes(uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::STOP_IN_ALL_MODES, channel, 0);
}

bool RoboteqController::setPosRelative(int32_t position, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_POS_RELATIVE, channel, position);
}

bool RoboteqController::setNextPosAbsolute(int32_t position, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_NEXT_POS_ABSOLUTE, channel, position);
}

bool RoboteqController::setNextPosRelative(int32_t position, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_NEXT_POS_RELATIVE, channel, position);
}

bool RoboteqController::setNextAcceleration(int32_t accel, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_NEXT_ACCELERATION, channel, accel);
}

bool RoboteqController::setNextDeceleration(int32_t decel, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_NEXT_DECELERATION, channel, decel);
}

bool RoboteqController::setNextVelocity(int32_t speed, uint8_t channel) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_NEXT_VELOCITY, channel, speed);
}

bool RoboteqController::setUserBoolVariable(uint32_t var, uint8_t nbvar) {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SET_USER_BOOL_VARIABLE, nbvar, var);
}

bool RoboteqController::saveConfigToFlash() {
  return comm_interface_->sdoDownload(roboteq::RuntimeCommand::SAVE_CONFIG_TO_FLASH, 0, 0);
}

// read methods
int16_t RoboteqController::readMotorAmps(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MOTOR_AMPS, channel)) *
         MOTOR_AMPS_READING_CONVERSION_FACTOR_;
}

int16_t RoboteqController::readActualMotorCommand(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ACTUAL_MOTOR_COMMAND, channel));
}

int16_t RoboteqController::readAppliedPowerLevel(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ACTUAL_POWER_LEVEL, channel));
}

int32_t RoboteqController::readEncoderMotorSpeed(uint8_t channel) {
  return reinterpret_cast<int32_t&>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ENCODER_MOTOR_SPEED, channel));
}

int32_t RoboteqController::readAbsoluteEncoderCount(uint8_t channel) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ABSOLUTE_ENCODER_COUNT, channel));
}

int32_t RoboteqController::readAbsoluteBrushlessCounter(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ABSOLUTE_BRUSHLESS_COUNTER, channel));
}

int32_t RoboteqController::readUserIntegerVariable(int32_t nbvar) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_USER_INTEGER_VARIABLE, nbvar));
}

int16_t RoboteqController::readEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
}

int32_t RoboteqController::readEncoderCountRelative(uint8_t channel) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ENCODER_COUNT_RELATIVE, channel));
}

int32_t RoboteqController::readBrushlessCountRelative(uint8_t channel) {
  return static_cast<int32_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_BRUSHLESS_COUNT_RELATIVE, channel));
}

int16_t RoboteqController::readBrushlessMotorSpeed(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED, channel));
}

int16_t RoboteqController::readBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  return static_cast<int16_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
}

int16_t RoboteqController::readBatteryAmps(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_BATTERY_AMPS, channel));
}

uint16_t RoboteqController::readInternalVoltages(uint8_t param) {
  return static_cast<uint16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_INTERNAL_VOLTAGES, param));
}

uint32_t RoboteqController::readAllDigitalInputs() {
  return static_cast<uint32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_ALL_DIGITAL_INPUTS));
}

int8_t RoboteqController::readCaseAndInternalTemperatures(uint8_t param) {
  return static_cast<int8_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_CASE_AND_INTERNAL_TEMPERATURES, param));
}

int16_t RoboteqController::readFeedback(uint8_t channel) {
  return static_cast<int16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_FEEDBACK, channel));
}

uint16_t RoboteqController::readStatusFlags() {
  return static_cast<uint16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_STATUS_FLAGS));
}

uint16_t RoboteqController::readFaultFlags() {
  return static_cast<uint16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_FAULT_FLAGS));
}

uint16_t RoboteqController::readCurrentDigitalOutputs() {
  return static_cast<uint16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_CURRENT_DIGITAL_OUTPUTS));
}

int32_t RoboteqController::readClosedLoopError(uint8_t channel) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_CLOSED_LOOP_ERROR, channel));
}

bool RoboteqController::readUserBooleanVariable(uint32_t nbvar) {
  return static_cast<bool>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_USER_BOOLEAN_VARIABLE, nbvar));
}

int32_t RoboteqController::readInternalSerialCommand(uint8_t channel) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_INTERNAL_SERIAL_COMMAND, channel));
}

int32_t RoboteqController::readInternalAnalogCommand(uint8_t channel) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_INTERNAL_ANALOG_COMMAND, channel));
}

int32_t RoboteqController::readInternalInternalPulseCommand(uint8_t channel) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_INTERNAL_PULSE_COMMAND, channel));
}

uint32_t RoboteqController::readTime(uint8_t param) {
  return static_cast<uint32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_TIME, param));
}

uint16_t RoboteqController::readSpektrumRadioCapture(uint8_t nb_capture) {
  return static_cast<uint16_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_SPEKTRUM_RADIO_CAPTURE, nb_capture));
}

uint8_t RoboteqController::readDestinationPositionReachedFlag(uint8_t channel) {
  return static_cast<uint8_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_DESTINATION_POSITION_REACHED_FLAG, channel));
}

int32_t RoboteqController::readMEMSAccelerometerAxis(uint8_t axis) {
  return static_cast<int32_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MEMS_ACCELEROMETER_AXIS, axis));
}

uint8_t RoboteqController::readMagsensorTrackDetect() {
  return static_cast<uint8_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MAGSENSOR_TRACK_DETECT));
}

int16_t RoboteqController::readMagsensorTrackPosition(uint8_t nb_pulse) {
  return static_cast<int16_t>(
      comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MAGSENSOR_TRACK_POSITION, nb_pulse));
}

uint8_t RoboteqController::readMagsensorMarkers(uint8_t nb_pulse) {
  return static_cast<uint8_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MAGSENSOR_MARKERS, nb_pulse));
}

uint16_t RoboteqController::readMagsensorStatus(uint8_t nb_pulse) {
  return static_cast<uint16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MAGSENSOR_STATUS, nb_pulse));
}

uint16_t RoboteqController::readMotorStatusFlags(uint8_t nb_pulse) {
  return static_cast<uint16_t>(comm_interface_->sdoUpload(roboteq::RuntimeQuery::READ_MOTOR_STATUS_FLAGS, nb_pulse));
}

}  // namespace roboteq
