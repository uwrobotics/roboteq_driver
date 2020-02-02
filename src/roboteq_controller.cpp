#include "roboteq_controller.hpp"

#include <stdlib.h>
#include <unistd.h>

#include <memory>

#include "i_comm.hpp"

namespace roboteq {

roboteq_controller::roboteq_controller(std::unique_ptr<i_comm> &&comm) : _comm(std::move(comm)) {}

// set methods
bool roboteq_controller::SetMotorCommand(int32_t command, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_MOTOR_COMMAND, channel, command));
  return sdo_response;
}

bool roboteq_controller::SetPosition(int32_t position, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_POSITION, channel, position));
  return sdo_response;
}

int16_t roboteq_controller::SetVelocity(int32_t speed, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_VELOCITY, channel, speed));
  return sdo_response;
}

int32_t roboteq_controller::SetEncoderCounter(int32_t counter, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_ENCODER_COUNTER, channel, counter));
  return sdo_response;
}

int32_t roboteq_controller::SetBrushlessCounter(int32_t counter, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_BRUSHLESS_COUNTER, channel, counter));
  return sdo_response;
}

int32_t roboteq_controller::SetUserIntVariable(int32_t var, uint8_t nbvar) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_USER_INT_VARIABLE, nbvar, var));
  return sdo_response;
}

int32_t roboteq_controller::SetAcceleration(int32_t accel, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_ACCELERATION, channel, accel));
  return sdo_response;
}

int32_t roboteq_controller::SetDeceleration(int32_t decel, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_DECELERATION, channel, decel));
  return sdo_response;
}

uint8_t roboteq_controller::SetAllDigitalOutBits(uint8_t out_bits) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_ALL_DIGITAL_OUT_BITS, 0, out_bits));
  return sdo_response;
}

uint8_t roboteq_controller::SetIndividualDigitalOutBits(uint8_t out_bits) {
  bool sdo_response = static_cast<bool>(
      _comm->sdo_download(roboteq::send_runtime_command::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0, out_bits));
  return sdo_response;
}

uint8_t roboteq_controller::ResetIndividualOutBits(uint8_t out_bits) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::RESET_INDIVIDUAL_OUT_BITS, 0, out_bits));
  return sdo_response;
}

uint8_t roboteq_controller::LoadHomeCounter(uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::LOAD_HOME_COUNTER, 0, channel));
  return sdo_response;
}

uint8_t roboteq_controller::EmergencyShutdown(uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::EMERGENCY_SHUTDOWN, 0, channel));
  return sdo_response;
}

uint8_t roboteq_controller::ReleaseShutdown(void) {
  bool sdo_response = static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::RELEASE_SHUTDOWN, 0, 0));
  return sdo_response;
}

uint8_t roboteq_controller::StopInAllModes(uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::STOP_IN_ALL_MODES, channel, 0));
  return sdo_response;
}

uint8_t roboteq_controller::SetPosRelative(int32_t position, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_POS_RELATIVE, channel, position));
  return sdo_response;
}

uint8_t roboteq_controller::SetNextPosAbsolute(int32_t position, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_POS_ABSOLUTE, channel, position));
  return sdo_response;
}

uint8_t roboteq_controller::SetNextPosRelative(int32_t position, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_POS_RELATIVE, channel, position));
  return sdo_response;
}

uint8_t roboteq_controller::SetNextAcceleration(int32_t accel, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_ACCELERATION, channel, accel));
  return sdo_response;
}

uint8_t roboteq_controller::SetNextDeceleration(int32_t decel, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_DECELERATION, channel, decel));
  return sdo_response;
}

uint8_t roboteq_controller::SetNextVelocity(int32_t speed, uint8_t channel) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_VELOCITY, channel, speed));
  return sdo_response;
}

uint32_t roboteq_controller::SetUserBoolVariable(uint32_t var, uint8_t nbvar) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SET_USER_BOOL_VARIABLE, nbvar, var));
  return sdo_response;
}

uint8_t roboteq_controller::SaveConfigToFlash(void) {
  bool sdo_response = static_cast<bool>(_comm->sdo_download(roboteq::send_runtime_command::SAVE_CONFIG_TO_FLASH, 0, 0));
  return sdo_response;
}

// read methods
int16_t roboteq_controller::ReadMotorAmps(uint8_t channel) {
  int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel));
  return sdo_response;
}

int16_t roboteq_controller::ReadActualMotorCommand(uint8_t channel) {
  int16_t sdo_response =
      static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ACTUAL_MOTOR_COMMAND, channel));
  return sdo_response;
}

int16_t roboteq_controller::ReadAppliedPowerLevel(uint8_t channel) {
  int16_t sdo_response =
      static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ACTUAL_POWER_LEVEL, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadEncoderMotorSpeed(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_MOTOR_SPEED, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadAbsoluteEncoderCount(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ABSOLUTE_ENCODER_COUNT, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadAbsoluteBrushlessCounter(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ABSOLUTE_BRUSHLESS_COUNTER, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadUserIntegerVariable(int32_t nbvar) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_USER_INTEGER_VARIABLE, nbvar));
  return sdo_response;
}

int16_t roboteq_controller::ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  int16_t sdo_response = static_cast<int16_t>(
      _comm->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadEncoderCountRelative(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_COUNT_RELATIVE, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadBrushlessCountRelative(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_COUNT_RELATIVE, channel));
  return sdo_response;
}

int16_t roboteq_controller::ReadBrushlessMotorSpeed(uint8_t channel) {
  int16_t sdo_response =
      static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED, channel));
  return sdo_response;
}

int16_t roboteq_controller::ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  int16_t sdo_response = static_cast<int16_t>(
      _comm->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
  return sdo_response;
}

int16_t roboteq_controller::ReadBatteryAmps(uint8_t channel) {
  int16_t sdo_response =
      static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BATTERY_AMPS, channel));
  return sdo_response;
}

uint16_t roboteq_controller::ReadInternalVoltages(uint8_t param) {
  uint16_t sdo_response =
      static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_VOLTAGES, param));
  return sdo_response;
}

uint32_t roboteq_controller::ReadAllDigitalInputs(void) {
  uint32_t sdo_response =
      static_cast<uint32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ALL_DIGITAL_INPUTS, 0));
  return sdo_response;
}

int8_t roboteq_controller::ReadCaseAndInternalTemperatures(uint8_t param) {
  int8_t sdo_response =
      static_cast<int8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_CASE_AND_INTERNAL_TEMPERATURES, param));
  return sdo_response;
}

int16_t roboteq_controller::ReadFeedback(uint8_t channel) {
  int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_FEEDBACK, channel));
  return sdo_response;
}

uint16_t roboteq_controller::ReadStatusFlags(void) {
  uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_STATUS_FLAGS, 0));
  return sdo_response;
}

uint16_t roboteq_controller::ReadFaultFlags(void) {
  uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_FAULT_FLAGS, 0));
  return sdo_response;
}

uint16_t roboteq_controller::ReadCurrentDigitalOutputs(void) {
  uint16_t sdo_response =
      static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_CURRENT_DIGITAL_OUTPUTS, 0));
  return sdo_response;
}

int32_t roboteq_controller::ReadClosedLoopError(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_CLOSED_LOOP_ERROR, channel));
  return sdo_response;
}

bool roboteq_controller::ReadUserBooleanVariable(uint32_t nbvar) {
  bool sdo_response =
      static_cast<bool>(_comm->sdo_upload(roboteq::send_runtime_query::READ_USER_BOOLEAN_VARIABLE, nbvar));
  return sdo_response;
}

int32_t roboteq_controller::ReadInternalSerialCommand(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_SERIAL_COMMAND, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadInternalAnalogCommand(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_ANALOG_COMMAND, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadInternalInternalPulseCommand(uint8_t channel) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_PULSE_COMMAND, channel));
  return sdo_response;
}

uint32_t roboteq_controller::ReadTime(uint8_t param) {
  uint32_t sdo_response = static_cast<uint32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_TIME, param));
  return sdo_response;
}

uint16_t roboteq_controller::ReadSpektrumRadioCapture(uint8_t nb_capture) {
  uint16_t sdo_response =
      static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_SPEKTRUM_RADIO_CAPTURE, nb_capture));
  return sdo_response;
}

uint8_t roboteq_controller::ReadDestinationPositionReachedFlag(uint8_t channel) {
  uint8_t sdo_response = static_cast<uint8_t>(
      _comm->sdo_upload(roboteq::send_runtime_query::READ_DESTINATION_POSITION_REACHED_FLAG, channel));
  return sdo_response;
}

int32_t roboteq_controller::ReadMEMSAccelerometerAxis(uint8_t axis) {
  int32_t sdo_response =
      static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MEMS_ACCELEROMETER_AXIS, axis));
  return sdo_response;
}

uint8_t roboteq_controller::ReadMagsensorTrackDetect(void) {
  uint8_t sdo_response =
      static_cast<uint8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_TRACK_DETECT, 0));
  return sdo_response;
}

int16_t roboteq_controller::ReadMagsensorTrackPosition(uint8_t nb_pulse) {
  int16_t sdo_response =
      static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_TRACK_POSITION, nb_pulse));
  return sdo_response;
}

uint8_t roboteq_controller::ReadMagsensorMarkers(uint8_t nb_pulse) {
  uint8_t sdo_response =
      static_cast<uint8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_MARKERS, nb_pulse));
  return sdo_response;
}

uint16_t roboteq_controller::ReadMagsensorStatus(uint8_t nb_pulse) {
  uint16_t sdo_response =
      static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_STATUS, nb_pulse));
  return sdo_response;
}

uint16_t roboteq_controller::ReadMotorStatusFlags(uint8_t nb_pulse) {
  uint16_t sdo_response =
      static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MOTOR_STATUS_FLAGS, nb_pulse));
  return sdo_response;
}

}  // namespace roboteq
