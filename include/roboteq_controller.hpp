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
  bool setMotorCommand(int32_t command, uint8_t channel);
  bool setPosition(int32_t position, uint8_t channel);
  bool setVelocity(int32_t speed, uint8_t channel);
  bool setEncoderCounter(int32_t counter, uint8_t channel);
  bool setBrushlessCounter(int32_t counter, uint8_t channel);
  bool setUserIntVariable(int32_t var, uint8_t nbvar);
  bool setAcceleration(int32_t accel, uint8_t channel);
  bool setDeceleration(int32_t decel, uint8_t channel);
  bool setAllDigitalOutBits(uint8_t out_bits);
  bool setIndividualDigitalOutBits(uint8_t out_bits);
  bool resetIndividualOutBits(uint8_t out_bits);
  bool loadHomeCounter(uint8_t channel);
  bool emergencyShutdown(void);
  bool releaseShutdown(void);
  bool stopInAllModes(uint8_t channel);
  bool setPosRelative(int32_t position, uint8_t channel);
  bool setNextPosAbsolute(int32_t position, uint8_t channel);
  bool setNextPosRelative(int32_t position, uint8_t channel);
  bool setNextAcceleration(int32_t accel, uint8_t channel);
  bool setNextDeceleration(int32_t decel, uint8_t channel);
  bool setNextVelocity(int32_t speed, uint8_t channel);
  bool setUserBoolVariable(uint32_t var, uint8_t nbvar);
  bool saveConfigToFlash(void);

  // read methods
  int16_t readMotorAmps(uint8_t channel);
  int16_t readActualMotorCommand(uint8_t channel);
  int16_t readAppliedPowerLevel(uint8_t channel);
  int32_t readEncoderMotorSpeed(uint8_t channel);
  int32_t readAbsoluteEncoderCount(uint8_t channel);
  int32_t readAbsoluteBrushlessCounter(uint8_t channel);
  int32_t readUserIntegerVariable(int32_t nbvar);
  int16_t readEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel);
  int32_t readEncoderCountRelative(uint8_t channel);
  int32_t readBrushlessCountRelative(uint8_t channel);
  int16_t readBrushlessMotorSpeed(uint8_t channel);
  int16_t readBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel);
  int16_t readBatteryAmps(uint8_t channel);
  uint16_t readInternalVoltages(uint8_t param);
  uint32_t readAllDigitalInputs(void);
  int8_t readCaseAndInternalTemperatures(uint8_t param);
  int16_t readFeedback(uint8_t channel);
  uint16_t readStatusFlags(void);
  uint16_t readFaultFlags(void);
  uint16_t readCurrentDigitalOutputs(void);
  int32_t readClosedLoopError(uint8_t channel);
  bool readUserBooleanVariable(uint32_t nbvar);
  int32_t readInternalSerialCommand(uint8_t channel);
  int32_t readInternalAnalogCommand(uint8_t channel);
  int32_t readInternalInternalPulseCommand(uint8_t channel);
  uint32_t readTime(uint8_t param);
  uint16_t readSpektrumRadioCapture(uint8_t nb_capture);
  uint8_t readDestinationPositionReachedFlag(uint8_t channel);
  int32_t readMEMSAccelerometerAxis(uint8_t axis);
  uint8_t readMagsensorTrackDetect(void);
  int16_t readMagsensorTrackPosition(uint8_t nb_pulse);
  uint8_t readMagsensorMarkers(uint8_t nb_pulse);
  uint16_t readMagsensorStatus(uint8_t nb_pulse);
  uint16_t readMotorStatusFlags(uint8_t nb_pulse);

 private:
  std::unique_ptr<i_comm> comm_interface_;

  static constexpr double MOTOR_AMPS_READING_CONVERSION_FACTOR{10.0};
};

}  // namespace roboteq
