#pragma once

namespace roboteq {

    class i_comm {

        public :

        i_comm();
        // i_comm() = delete;
        virtual ~i_comm(){}

        virtual int read(void);
        virtual int write(void);

        // set methods
        virtual void SetParameters(int16_t _velocity, long _accel, long _decel, uint8_t _channel)=0;
        virtual void SetPosition(long _position, uint8_t _channel)=0;
        virtual void SetVelocity(uint16_t _speed, uint8_t _channel)=0;
        virtual void SetEncoderCounter(long _counter, uint8_t _channel)=0;
        virtual void SetBrushlessCounter(long _counter, uint8_t _channel)=0;
        virtual void SetUserIntVariable(long _var, uint8_t _nb_var)=0;
        virtual void SetAcceleration(long _accel, uint8_t _channel)=0;
        virtual void SetDeceleration(long _decel, uint8_t _channel)=0;
        virtual void SetAllDigitalOutBits(uint8_t _out_bits)=0;
        virtual void SetIndividualDigitalOutBits(uint8_t _out_bits)=0;
        virtual void ResetIndividualOutBits(uint8_t _out_bits)=0;
        virtual void LoadHomeCounter(uint8_t _channel)=0;
        virtual void EmergencyShutdown(uint8_t _param)=0;
        virtual void ReleaseShutdown(uint8_t _param)=0;
        virtual void StopInAllModes(uint8_t _channel)=0;
        virtual void SetPosRelative(long _position, uint8_t _channel)=0;
        virtual void SetNextPosAbsolute(long _position, uint8_t _channel)=0;
        virtual void SetNextPosRelative(long _position, uint8_t _channel)=0;
        virtual void SetNextAcceleration(long _accel, uint8_t _channel)=0;
        virtual void SetNextDeceleration(long _decel, uint8_t _channel)=0;
        virtual void SetNextVelocity(uint16_t _speed, uint8_t _channel)=0;
        virtual void SetUserBoolVariable(long _var, uint8_t _nb_var)=0;
        virtual void SaveConfigToFlash(void)=0;

        // read methods
        virtual int16_t ReadMotorAmps(uint8_t _channel)=0;
        virtual int16_t ReadActualMotorCommand(uint8_t _channel)=0;
        virtual int16_t ReadAppliedPowerLevel(uint8_t _channel)=0;
        virtual int16_t ReadEncoderMotorSpeed(uint8_t _channel)=0;
        virtual int16_t ReadAbsoluteEncoderCount(uint8_t _channel)=0;
        virtual int16_t ReadAbsoluteBrushlessCounter(uint8_t _channel)=0;
        virtual int16_t ReadUserIntegerVariable(uint8_t _nb_var)=0;
        virtual int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t _channel)=0;
        virtual int16_t ReadEncoderCountRelative(uint8_t _channel)=0;
        virtual int16_t ReadBrushlessCountRelative(uint8_t _channel)=0;
        virtual int16_t ReadBrushlessMotorSpeed(uint8_t _channel)=0;
        virtual int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t _channel)=0;
        virtual int16_t ReadBatteryAmps(uint8_t _channel)=0;
        virtual uint16_t ReadInternalVoltages(uint8_t _param)=0;
        virtual uint32_t ReadAllDigitalInputs(void)=0;
        virtual int8_t ReadCaseAndInternalTemperatures(uint8_t _param)=0;
        virtual int16_t ReadFeedback(uint8_t _channel)=0;
        virtual uint8_t ReadStatusFlags(void)=0;
        virtual uint8_t ReadFaultFlags(void)=0;
        virtual uint8_t ReadCurrentDigitalOutputs(void)=0;
        virtual int32_t ReadClosedLoopError(uint8_t _channel)=0;
        virtual int32_t ReadUserBooleanVariable(uint8_t _nb_var)=0;
        virtual int32_t ReadInternalSerialCommand(uint8_t _channel)=0;
        virtual uint32_t ReadTime(void)=0;
        virtual uint16_t ReadSpektrumRadioCapture(uint8_t _nb_capture)=0;
        virtual uint8_t ReadDestinationPositionReachedFlag(uint8_t _channel)=0;
        virtual int32_t ReadMEMSAccelerometerAxis(uint8_t _axis)=0;
        virtual uint16_t ReadMagsensorTrackDetect(void)=0;
        virtual uint8_t ReadMagsensorTrackPosition(void)=0;
        virtual uint8_t ReadMagsensorMarkers(void)=0;
        virtual uint8_t ReadMagsensorStatus(void)=0;
        virtual uint8_t ReadMotorStatusFlags(void)=0;
        virtual int32_t ReadIndividualDigitalInputs(uint8_t _input)=0;
        virtual int16_t ReadAnalogInputs(uint8_t _input)=0;
        virtual int16_t ReadAnalogInputsConverted(uint8_t _input)=0;
        virtual int16_t ReadPulseInputs(uint8_t _input)=0;
        virtual int16_t ReadPulseInputsConverted(uint8_t _input)=0;

        private :

    };

} //namespace roboteq