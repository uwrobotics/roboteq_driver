#pragma once

#include <memory> 

#include "i_comm.hpp"

namespace roboteq {

    class roboteq_controller {

        public :
        explicit roboteq_controller(std::unique_ptr<i_comm> &&comm);
        roboteq_controller ( const roboteq_controller & ) = delete;
        roboteq_controller & operator= ( const roboteq_controller& ) = delete;
        ~roboteq_controller(){}

        // set methods
        void SetParameters(int16_t velocity, long accel, long decel, uint8_t channel);	
        void SetPosition(long position, uint8_t channel);	

        void SetVelocity(uint16_t speed, uint8_t channel){
            _comm->download(roboteq::runtime_command::SET_VELOCITY, speed, channel);
        };
        	
        void SetEncoderCounter(long counter, uint8_t channel);	
        void SetBrushlessCounter(long counter, uint8_t channel);	
        void SetUserIntVariable(long var, uint8_t nbvar);	
        void SetAcceleration(long accel, uint8_t channel);	
        void SetDeceleration(long decel, uint8_t channel);	
        void SetAllDigitalOutBits(uint8_t out_bits);	
        void SetIndividualDigitalOutBits(uint8_t out_bits);	
        void ResetIndividualOutBits(uint8_t out_bits);	
        void LoadHomeCounter(uint8_t channel);	
        void EmergencyShutdown(uint8_t param);	
        void ReleaseShutdown(uint8_t param);	
        void StopInAllModes(uint8_t channel);	
        void SetPosRelative(long position, uint8_t channel);	
        void SetNextPosAbsolute(long position, uint8_t channel);	
        void SetNextPosRelative(long position, uint8_t channel);	
        void SetNextAcceleration(long accel, uint8_t channel);	
        void SetNextDeceleration(long decel, uint8_t channel);	
        void SetNextVelocity(uint16_t speed, uint8_t channel);	
        void SetUserBoolVariable(long var, uint8_t nbvar);	
        void SaveConfigToFlash(void);	

        // read methods	
        int16_t ReadMotorAmps(uint8_t channel);	
        int16_t ReadActualMotorCommand(uint8_t channel);	
        int16_t ReadAppliedPowerLevel(uint8_t channel);	
        int16_t ReadEncoderMotorSpeed(uint8_t channel);	
        int16_t ReadAbsoluteEncoderCount(uint8_t channel);	
        int16_t ReadAbsoluteBrushlessCounter(uint8_t channel);	
        int16_t ReadUserIntegerVariable(uint8_t nbvar);	
        int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel);	
        int16_t ReadEncoderCountRelative(uint8_t channel);	
        int16_t ReadBrushlessCountRelative(uint8_t channel);	
        int16_t ReadBrushlessMotorSpeed(uint8_t channel);	
        int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel);	
        int16_t ReadBatteryAmps(uint8_t channel);	
        uint16_t ReadInternalVoltages(uint8_t param);	
        uint32_t ReadAllDigitalInputs(void);	
        int8_t ReadCaseAndInternalTemperatures(uint8_t param);	
        int16_t ReadFeedback(uint8_t channel);	
        uint8_t ReadStatusFlags(void);	
        uint8_t ReadFaultFlags(void);	
        uint8_t ReadCurrentDigitalOutputs(void);	
        int32_t ReadClosedLoopError(uint8_t channel);	
        int32_t ReadUserBooleanVariable(uint8_t nbvar);	
        int32_t ReadInternalSerialCommand(uint8_t channel);	
        uint32_t ReadTime(void);	
        uint16_t ReadSpektrumRadioCapture(uint8_t nb_capture);	
        uint8_t ReadDestinationPositionReachedFlag(uint8_t channel);	
        int32_t ReadMEMSAccelerometerAxis(uint8_t axis);	
        uint16_t ReadMagsensorTrackDetect(void);	
        uint8_t ReadMagsensorTrackPosition(void);	
        uint8_t ReadMagsensorMarkers(void);	
        uint8_t ReadMagsensorStatus(void);	
        uint8_t ReadMotorStatusFlags(void);	
        int32_t ReadIndividualDigitalInputs(uint8_t input);	
        int16_t ReadAnalogInputs(uint8_t input);	
        int16_t ReadAnalogInputsConverted(uint8_t input);	
        int16_t ReadPulseInputs(uint8_t input);	
        int16_t ReadPulseInputsConverted(uint8_t input);

        // private :
        std::unique_ptr<i_comm> _comm;
        
    };

} //namespace roboteq 

