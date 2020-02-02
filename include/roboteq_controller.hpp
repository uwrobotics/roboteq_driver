#pragma once

#include <iostream>
#include <memory>

#include "i_comm.hpp"

namespace roboteq {

class roboteq_controller {
 public:
  explicit roboteq_controller(std::unique_ptr<i_comm>&& comm);
  roboteq_controller(const roboteq_controller&) = delete;
  roboteq_controller& operator=(const roboteq_controller&) = delete;
  ~roboteq_controller() {}

  // set methods
  bool SetMotorCommand(int32_t command, uint8_t channel);

  bool SetPosition(int32_t position, uint8_t channel);

  int16_t SetVelocity(int32_t speed, uint8_t channel);

  int32_t SetEncoderCounter(int32_t counter, uint8_t channel);

  int32_t SetBrushlessCounter(int32_t counter, uint8_t channel);

  int32_t SetUserIntVariable(int32_t var, uint8_t nbvar);

  int32_t SetAcceleration(int32_t accel, uint8_t channel);

  int32_t SetDeceleration(int32_t decel, uint8_t channel);

  uint8_t SetAllDigitalOutBits(uint8_t out_bits);

  uint8_t SetIndividualDigitalOutBits(uint8_t out_bits);

  uint8_t ResetIndividualOutBits(uint8_t out_bits);

  uint8_t LoadHomeCounter(uint8_t channel);

  uint8_t EmergencyShutdown(uint8_t channel);

  uint8_t ReleaseShutdown(void);

  uint8_t StopInAllModes(uint8_t channel);

  uint8_t SetPosRelative(int32_t position, uint8_t channel);

  uint8_t SetNextPosAbsolute(int32_t position, uint8_t channel);

  uint8_t SetNextPosRelative(int32_t position, uint8_t channel);

  uint8_t SetNextAcceleration(int32_t accel, uint8_t channel);

  uint8_t SetNextDeceleration(int32_t decel, uint8_t channel);

  uint8_t SetNextVelocity(int32_t speed, uint8_t channel);

  uint32_t SetUserBoolVariable(uint32_t var, uint8_t nbvar);

  uint8_t SaveConfigToFlash(void);

  // read methods
  int16_t ReadMotorAmps(uint8_t channel);

  int16_t ReadActualMotorCommand(uint8_t channel);

  int16_t ReadAppliedPowerLevel(uint8_t channel);

  int32_t ReadEncoderMotorSpeed(uint8_t channel);

  int32_t ReadAbsoluteEncoderCount(uint8_t channel);

  int32_t ReadAbsoluteBrushlessCounter(uint8_t channel);

  int32_t ReadUserIntegerVariable(int32_t nbvar);

  int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel);

  int32_t ReadEncoderCountRelative(uint8_t channel);

  int32_t ReadBrushlessCountRelative(uint8_t channel);

  int16_t ReadBrushlessMotorSpeed(uint8_t channel);

  int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel);

  int16_t ReadBatteryAmps(uint8_t channel);

  uint16_t ReadInternalVoltages(uint8_t param);

  uint32_t ReadAllDigitalInputs(void);

  int8_t ReadCaseAndInternalTemperatures(uint8_t param);

  int16_t ReadFeedback(uint8_t channel);

  uint16_t ReadStatusFlags(void);

  uint16_t ReadFaultFlags(void);

  uint16_t ReadCurrentDigitalOutputs(void);

  int32_t ReadClosedLoopError(uint8_t channel);

  bool ReadUserBooleanVariable(uint32_t nbvar);

  int32_t ReadInternalSerialCommand(uint8_t channel);

  int32_t ReadInternalAnalogCommand(uint8_t channel);

  int32_t ReadInternalInternalPulseCommand(uint8_t channel);

  uint32_t ReadTime(uint8_t param);

  uint16_t ReadSpektrumRadioCapture(uint8_t nb_capture);

  uint8_t ReadDestinationPositionReachedFlag(uint8_t channel);

  int32_t ReadMEMSAccelerometerAxis(uint8_t axis);

  uint8_t ReadMagsensorTrackDetect(void);

  int16_t ReadMagsensorTrackPosition(uint8_t nb_pulse);

  uint8_t ReadMagsensorMarkers(uint8_t nb_pulse);

  uint16_t ReadMagsensorStatus(uint8_t nb_pulse);

  uint16_t ReadMotorStatusFlags(uint8_t nb_pulse);

  // private :
  std::unique_ptr<i_comm> _comm;
};

}  // namespace roboteq
