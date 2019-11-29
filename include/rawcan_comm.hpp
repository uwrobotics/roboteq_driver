#pragma once

#include <linux/can.h>

namespace roboteq { 

class rawcan_comm : public roboteq {

rawcan_comm(canid_t _roboteq_can_id, std::string _ifname);
~rawcan_comm(){}

// set methods
void SetParameters(int16_t _velocity, long _accel, long _decel, uint8_t _channel) override;
void SetPosition(long _position, uint8_t _channel) override;
void SetVelocity(uint16_t _speed, uint8_t _channel) override;
void SetEncoderCounter(long _counter, uint8_t _channel) override;
void SetBrushlessCounter(long _counter, uint8_t _channel) override;
void SetUserIntVariable(long _var, uint8_t _nb_var) override;
void SetAcceleration(long _accel, uint8_t _channel) override;
void SetDeceleration(long _decel, uint8_t _channel) override;
void SetAllDigitalOutBits(uint8_t _out_bits) override;
void SetIndividualDigitalOutBits(uint8_t _out_bits) override;
void ResetIndividualOutBits(uint8_t _out_bits) override;
void LoadHomeCounter(uint8_t _channel) override;
void EmergencyShutdown(uint8_t _param) override;
void ReleaseShutdown(uint8_t _param) override;
void StopInAllModes(uint8_t _channel) override;
void SetPosRelative(long _position, uint8_t _channel) override;
void SetNextPosAbsolute(long _position, uint8_t _channel) override;
void SetNextPosRelative(long _position, uint8_t _channel) override;
void SetNextAcceleration(long _accel, uint8_t _channel) override;
void SetNextDeceleration(long _decel, uint8_t _channel) override;
void SetNextVelocity(uint16_t _speed, uint8_t _channel) override;
void SetUserBoolVariable(long _var, uint8_t _nb_var) override;
void SaveConfigToFlash(void) override;

// read methods
int16_t ReadActualMotorCommand(uint8_t _channel) override;
int16_t ReadAppliedPowerLevel(uint8_t _channel) override;
int16_t ReadEncoderMotorSpeed(uint8_t _channel) override;
int16_t ReadAbsoluteEncoderCount(uint8_t _channel) override;
int16_t ReadAbsoluteBrushlessCounter(uint8_t _channel) override;
int16_t ReadUserIntegerVariable(uint8_t _nb_var) override;
int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t _channel) override;
int16_t ReadEncoderCountRelative(uint8_t _channel) override;
int16_t ReadBrushlessCountRelative(uint8_t _channel) override;
int16_t ReadBrushlessMotorSpeed(uint8_t _channel) override;
int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t _channel) override;
int16_t ReadBatteryAmps(uint8_t _channel) override;
uint16_t ReadInternalVoltages(uint8_t _param) override;
uint32_t ReadAllDigitalInputs(void) override;
int8_t ReadCaseAndInternalTemperatures(uint8_t _param) override;
int16_t ReadFeedback(uint8_t _channel) override;
uint8_t ReadStatusFlags(void) override;
uint8_t ReadFaultFlags(void) override;
uint8_t ReadCurrentDigitalOutputs(void) override;
int32_t ReadClosedLoopError(uint8_t _channel) override;
int32_t ReadUserBooleanVariable(uint8_t _nb_var) override;
int32_t ReadInternalSerialCommand(uint8_t _channel) override;
uint32_t ReadTime(void) override;
uint16_t ReadSpektrumRadioCapture(uint8_t _nb_capture) override;
uint8_t ReadDestinationPositionReachedFlag(uint8_t _channel) override;
int32_t ReadMEMSAccelerometerAxis(uint8_t _axis) override;
uint16_t ReadMagsensorTrackDetect(void) override;
uint8_t ReadMagsensorTrackPosition(void) override;
uint8_t ReadMagsensorMarkers(void) override;
uint8_t ReadMagsensorStatus(void) override;
uint8_t ReadMotorStatusFlags(void) override;
int32_t ReadIndividualDigitalInputs(uint8_t _input) override;
int16_t ReadAnalogInputs(uint8_t _input) override override;
int16_t ReadAnalogInputsConverted(uint8_t _input) override;
int16_t ReadPulseInputs(uint8_t _input) override;
int16_t ReadPulseInputsConverted(uint8_t _input) override;

private :
const _roboteq_can_id;
int _socket_handle;



} 

} // namespace roboteq