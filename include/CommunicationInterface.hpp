#pragma once

namespace roboteq {

enum class RuntimeCommand {
  SET_MOTOR_COMMAND = 0,
  SET_POSITION,
  SET_VELOCITY,
  SET_ENCODER_COUNTER,
  SET_BRUSHLESS_COUNTER,
  SET_USER_INT_VARIABLE,
  SET_ACCELERATION,
  SET_DECELERATION,
  SET_ALL_DIGITAL_OUT_BITS,
  SET_INDIVIDUAL_DIGITAL_OUT_BITS,
  RESET_INDIVIDUAL_OUT_BITS,
  LOAD_HOME_COUNTER,
  EMERGENCY_SHUTDOWN,
  RELEASE_SHUTDOWN,
  STOP_IN_ALL_MODES,
  SET_POS_RELATIVE,
  SET_NEXT_POS_ABSOLUTE,
  SET_NEXT_POS_RELATIVE,
  SET_NEXT_ACCELERATION,
  SET_NEXT_DECELERATION,
  SET_NEXT_VELOCITY,
  SET_USER_BOOL_VARIABLE,
  SAVE_CONFIG_TO_FLASH,
};

enum class RuntimeQuery {

  READ_MOTOR_AMPS = 0,
  READ_ACTUAL_MOTOR_COMMAND,
  READ_ACTUAL_POWER_LEVEL,
  READ_ENCODER_MOTOR_SPEED,
  READ_ABSOLUTE_ENCODER_COUNT,
  READ_ABSOLUTE_BRUSHLESS_COUNTER,
  READ_USER_INTEGER_VARIABLE,
  READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED,
  READ_ENCODER_COUNT_RELATIVE,
  READ_BRUSHLESS_COUNT_RELATIVE,
  READ_BRUSHLESS_MOTOR_SPEED,
  READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED,
  READ_BATTERY_AMPS,
  READ_INTERNAL_VOLTAGES,
  READ_ALL_DIGITAL_INPUTS,
  READ_CASE_AND_INTERNAL_TEMPERATURES,
  READ_FEEDBACK,
  READ_STATUS_FLAGS,
  READ_FAULT_FLAGS,
  READ_CURRENT_DIGITAL_OUTPUTS,
  READ_CLOSED_LOOP_ERROR,
  READ_USER_BOOLEAN_VARIABLE,
  READ_INTERNAL_SERIAL_COMMAND,
  READ_INTERNAL_ANALOG_COMMAND,
  READ_INTERNAL_PULSE_COMMAND,
  READ_TIME,
  READ_SPEKTRUM_RADIO_CAPTURE,
  READ_DESTINATION_POSITION_REACHED_FLAG,
  READ_MEMS_ACCELEROMETER_AXIS,
  READ_MAGSENSOR_TRACK_DETECT,
  READ_MAGSENSOR_TRACK_POSITION,
  READ_MAGSENSOR_MARKERS,
  READ_MAGSENSOR_STATUS,
  READ_MOTOR_STATUS_FLAGS,
  READ_INDIVIDUAL_DIGITAL_INPUTS,
  READ_ANALOG_INPUTS,
  READ_ANALOG_INPUTS_CONVERTED,
  READ_PULSE_INPUTS,
  READ_PULSE_INPUTS_CONVERTED,
};

class CommunicationInterface {
 public:
  CommunicationInterface() = default;
  CommunicationInterface(CommunicationInterface&&) = default;
  CommunicationInterface& operator=(CommunicationInterface&&) = default;
  CommunicationInterface(const CommunicationInterface&) = default;
  CommunicationInterface& operator=(const CommunicationInterface&) = default;
  virtual ~CommunicationInterface() = default;

  //  virtual bool sendCommand(RuntimeCommand command, uint8_t subindex = 0, uint32_t data = 0) = 0;
  //  virtual uint32_t sendQuery(RuntimeQuery query, uint8_t subindex = 0) = 0;
};

}  // namespace roboteq
