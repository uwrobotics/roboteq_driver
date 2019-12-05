#pragma once

namespace roboteq {

    enum class runtime_command {

        SET_MOTOR_COMMAND =0,
        SET_POSITION,
        SET_VELOCITY

    };

    enum class runtime_query {

        READ_MOTOR_AMPS =0,
        READ_ACTUAL_MOTOR_COMMAND,
        READ_ACTUAL_POWER_LEVEL,

    };


    class i_comm {

        public :

        i_comm(){};
        virtual ~i_comm(){}

        virtual bool write(runtime_command command, uint8_t argument1, uint32_t argument2)=0;
        virtual uint32_t read(runtime_query command, uint8_t argument1)=0;

        // // set methods
        // virtual void SetParameters(int16_t velocity, long accel, long decel, uint8_t channel) =0;
        // virtual void SetPosition(long position, uint8_t channel) =0;
        // virtual void SetVelocity(uint16_t speed, uint8_t channel) =0;
        // virtual void SetEncoderCounter(long counter, uint8_t channel) =0;
        // virtual void SetBrushlessCounter(long counter, uint8_t channel) =0;
        // virtual void SetUserIntVariable(long var, uint8_t nbvar) =0;
        // virtual void SetAcceleration(long accel, uint8_t channel) =0;
        // virtual void SetDeceleration(long decel, uint8_t channel) =0;
        // virtual void SetAllDigitalOutBits(uint8_t out_bits) =0;
        // virtual void SetIndividualDigitalOutBits(uint8_t out_bits) =0;
        // virtual void ResetIndividualOutBits(uint8_t out_bits) =0;
        // virtual void LoadHomeCounter(uint8_t channel) =0;
        // virtual void EmergencyShutdown(uint8_t param) =0;
        // virtual void ReleaseShutdown(uint8_t param) =0;
        // virtual void StopInAllModes(uint8_t channel) =0;
        // virtual void SetPosRelative(long position, uint8_t channel) =0;
        // virtual void SetNextPosAbsolute(long position, uint8_t channel) =0;
        // virtual void SetNextPosRelative(long position, uint8_t channel) =0;
        // virtual void SetNextAcceleration(long accel, uint8_t channel) =0;
        // virtual void SetNextDeceleration(long decel, uint8_t channel) =0;
        // virtual void SetNextVelocity(uint16_t speed, uint8_t channel) =0;
        // virtual void SetUserBoolVariable(long var, uint8_t nbvar) =0;
        // virtual void SaveConfigToFlash(void) =0;

        // // read methods
        // virtual int16_t ReadMotorAmps(uint8_t channel) =0;
        // virtual int16_t ReadActualMotorCommand(uint8_t channel) =0;
        // virtual int16_t ReadAppliedPowerLevel(uint8_t channel) =0;
        // virtual int16_t ReadEncoderMotorSpeed(uint8_t channel) =0;
        // virtual int16_t ReadAbsoluteEncoderCount(uint8_t channel) =0;
        // virtual int16_t ReadAbsoluteBrushlessCounter(uint8_t channel) =0;
        // virtual int16_t ReadUserIntegerVariable(uint8_t nbvar) =0;
        // virtual int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel) =0;
        // virtual int16_t ReadEncoderCountRelative(uint8_t channel) =0;
        // virtual int16_t ReadBrushlessCountRelative(uint8_t channel) =0;
        // virtual int16_t ReadBrushlessMotorSpeed(uint8_t channel) =0;
        // virtual int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel) =0;
        // virtual int16_t ReadBatteryAmps(uint8_t channel) =0;
        // virtual uint16_t ReadInternalVoltages(uint8_t param) =0;
        // virtual uint32_t ReadAllDigitalInputs(void) =0;
        // virtual int8_t ReadCaseAndInternalTemperatures(uint8_t param) =0;
        // virtual int16_t ReadFeedback(uint8_t channel) =0;
        // virtual uint8_t ReadStatusFlags(void) =0;
        // virtual uint8_t ReadFaultFlags(void) =0;
        // virtual uint8_t ReadCurrentDigitalOutputs(void) =0;
        // virtual int32_t ReadClosedLoopError(uint8_t channel) =0;
        // virtual int32_t ReadUserBooleanVariable(uint8_t nbvar) =0;
        // virtual int32_t ReadInternalSerialCommand(uint8_t channel) =0;
        // virtual uint32_t ReadTime(void) =0;
        // virtual uint16_t ReadSpektrumRadioCapture(uint8_t nb_capture) =0;
        // virtual uint8_t ReadDestinationPositionReachedFlag(uint8_t channel) =0;
        // virtual int32_t ReadMEMSAccelerometerAxis(uint8_t axis) =0;
        // virtual uint16_t ReadMagsensorTrackDetect(void) =0;
        // virtual uint8_t ReadMagsensorTrackPosition(void) =0;
        // virtual uint8_t ReadMagsensorMarkers(void) =0;
        // virtual uint8_t ReadMagsensorStatus(void) =0;
        // virtual uint8_t ReadMotorStatusFlags(void) =0;
        // virtual int32_t ReadIndividualDigitalInputs(uint8_t input) =0;
        // virtual int16_t ReadAnalogInputs(uint8_t input) =0;
        // virtual int16_t ReadAnalogInputsConverted(uint8_t input) =0;
        // virtual int16_t ReadPulseInputs(uint8_t input) =0;
        // virtual int16_t ReadPulseInputsConverted(uint8_t input) =0;
        
        private :

    };

} //namespace roboteq