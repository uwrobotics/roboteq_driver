#pragma once

namespace roboteq {

class i_comm {

public :

virtual i_comm() = 0;
virtual ~i_comm(){}

virtual read(void);
virtual write(void);

// set methods
virtual void SetParameters(int16_t _velocity, long _accel, long _decel, uint8_t _channel);
virtual void SetPosition(long _position, uint8_t _channel);
virtual void SetVelocity(uint16_t _speed, uint8_t _channel);
virtual void SetEncoderCounter(long _counter, uint8_t _channel);
virtual void SetBrushlessCounter(long _counter, uint8_t _channel);
virtual void SetUserIntVariable(long _var, uint8_t _nb_var);
virtual void SetAcceleration(long _accel, uint8_t _channel);
virtual void SetDeceleration(long _decel, uint8_t _channel);
virtual void SetAllDigitalOutBits(uint8_t _out_bits);
virtual void SetIndividualDigitalOutBits(uint8_t _out_bits);
virtual void ResetIndividualOutBits(uint8_t _out_bits);
virtual void LoadHomeCounter(uint8_t _channel);
virtual void EmergencyShutdown(uint8_t _param);
virtual void ReleaseShutdown(uint8_t _param);
virtual void StopInAllModes(uint8_t _channel);
virtual void SetPosRelative(long _position, uint8_t _channel);
virtual void SetNextPosAbsolute(long _position, uint8_t _channel);
virtual void SetNextPosRelative(long _position, uint8_t _channel);
virtual void SetNextAcceleration(long _accel, uint8_t _channel);
virtual void SetNextDeceleration(long _decel, uint8_t _channel);
virtual void SetNextVelocity(uint16_t _speed, uint8_t _channel);
virtual void SetUserBoolVariable(long _var, uint8_t _nb_var);
virtual void SaveConfigToFlash(void);

// read methods
virtual int16_t ReadMotorAmps(uint8_t _channel);
virtual int16_t ReadActualMotorCommand(uint8_t _channel);
virtual int16_t ReadAppliedPowerLevel(uint8_t _channel);
virtual int16_t ReadEncoderMotorSpeed(uint8_t _channel);
virtual int16_t ReadAbsoluteEncoderCount(uint8_t _channel);
virtual int16_t ReadAbsoluteBrushlessCounter(uint8_t _channel);
virtual int16_t ReadUserIntegerVariable(uint8_t _nb_var);
virtual int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t _channel);
virtual int16_t ReadEncoderCountRelative(uint8_t _channel);
virtual int16_t ReadBrushlessCountRelative(uint8_t _channel);
virtual int16_t ReadBrushlessMotorSpeed(uint8_t _channel);
virtual int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t _channel);
virtual int16_t ReadBatteryAmps(uint8_t _channel);
virtual uint16_t ReadInternalVoltages(uint8_t _param);
virtual uint32_t ReadAllDigitalInputs(void);
virtual int8_t ReadCaseAndInternalTemperatures(uint8_t _param);
virtual int16_t ReadFeedback(uint8_t _channel);
virtual uint8_t ReadStatusFlags(void);
virtual uint8_t ReadFaultFlags(void);
virtual uint8_t ReadCurrentDigitalOutputs(void);
virtual int32_t ReadClosedLoopError(uint8_t _channel);
virtual int32_t ReadUserBooleanVariable(uint8_t _nb_var);
virtual int32_t ReadInternalSerialCommand(uint8_t _channel);
virtual uint32_t ReadTime(void);
virtual uint16_t ReadSpektrumRadioCapture(uint8_t _nb_capture);
virtual uint8_t ReadDestinationPositionReachedFlag(uint8_t _channel);
virtual int32_t ReadMEMSAccelerometerAxis(uint8_t _axis);
virtual uint16_t ReadMagsensorTrackDetect(void);
virtual uint8_t ReadMagsensorTrackPosition(void);
virtual uint8_t ReadMagsensorMarkers(void);
virtual uint8_t ReadMagsensorStatus(void);
virtual uint8_t ReadMotorStatusFlags(void);
virtual int32_t ReadIndividualDigitalInputs(uint8_t _input);
virtual int16_t ReadAnalogInputs(uint8_t _input);
virtual int16_t ReadAnalogInputsConverted(uint8_t _input);
virtual int16_t ReadPulseInputs(uint8_t _input);
virtual int16_t ReadPulseInputsConverted(uint8_t _input);

private :

} // class i_comms

} //namespace roboteq