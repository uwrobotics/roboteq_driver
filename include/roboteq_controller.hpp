#pragma once

#include <memory> 
#include <iostream>

#include "i_comm.hpp"

namespace roboteq {

    class roboteq_controller {

        public :
        explicit roboteq_controller(std::unique_ptr<i_comm> &&comm);
        roboteq_controller ( const roboteq_controller & ) = delete;
        roboteq_controller & operator= ( const roboteq_controller& ) = delete;
        ~roboteq_controller(){}

        // set methods
        void SetPosition(long position, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_POSITION, channel, position);
        };

        void SetVelocity(uint16_t speed, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_VELOCITY, channel, speed);
            std::cout << "Velocity!" << std::endl;
        };
        	
        void SetEncoderCounter(long counter, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_ENCODER_COUNTER, channel, counter);
        };
        
        void SetBrushlessCounter(long counter, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_BRUSHLESS_COUNTER, channel, counter);
        };

        void SetUserIntVariable(long var, uint8_t nbvar){
            _comm->download(roboteq::send_runtime_command::SET_USER_INT_VARIABLE, nbvar, var);
        };

        void SetAcceleration(long accel, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_ACCELERATION, channel, accel);
        };

        void SetDeceleration(long decel, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_DECELERATION, channel, decel);
        };

        void SetAllDigitalOutBits(uint8_t out_bits){
            _comm->download(roboteq::send_runtime_command::SET_ALL_DIGITAL_OUT_BITS, 0, out_bits);
        };

        void SetIndividualDigitalOutBits(uint8_t out_bits){
            _comm->download(roboteq::send_runtime_command::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0, out_bits);
        };

        void ResetIndividualOutBits(uint8_t out_bits){
            _comm->download(roboteq::send_runtime_command::RESET_INDIVIDUAL_OUT_BITS, 0, out_bits);
        };

        void LoadHomeCounter(uint8_t channel){
            _comm->download(roboteq::send_runtime_command::LOAD_HOME_COUNTER, 0, 0);
        };

        void EmergencyShutdown(uint8_t param){
            _comm->download(roboteq::send_runtime_command::EMERGENCY_SHUTDOWN, 0, 0);
        };

        void ReleaseShutdown(uint8_t param){
            _comm->download(roboteq::send_runtime_command::RELEASE_SHUTDOWN, 0, 0);
        };

        void StopInAllModes(uint8_t channel){
            _comm->download(roboteq::send_runtime_command::STOP_IN_ALL_MODES, channel, 0);
        };

        void SetPosRelative(long position, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_POS_RELATIVE, channel, position);
        };

        void SetNextPosAbsolute(long position, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_NEXT_POS_ABSOLUTE, channel, position);
        };

        void SetNextPosRelative(long position, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_NEXT_POS_RELATIVE, channel, position);
        };

        void SetNextAcceleration(long accel, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_NEXT_ACCELERATION, channel, accel);
        };

        void SetNextDeceleration(long decel, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_NEXT_DECELERATION, channel, decel);
        };

        void SetNextVelocity(uint16_t speed, uint8_t channel){
            _comm->download(roboteq::send_runtime_command::SET_NEXT_VELOCITY, channel, speed);
        };

        void SetUserBoolVariable(long var, uint8_t nbvar){
            _comm->download(roboteq::send_runtime_command::SET_USER_BOOL_VARIABLE, nbvar, var);
        };

        void SaveConfigToFlash(void){
            _comm->download(roboteq::send_runtime_command::SAVE_CONFIG_TO_FLASH, 0, 0);
        };	

        // read methods	
        int16_t ReadMotorAmps(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadActualMotorCommand(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadAppliedPowerLevel(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadEncoderMotorSpeed(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	
        
        int16_t ReadAbsoluteEncoderCount(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadAbsoluteBrushlessCounter(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadUserIntegerVariable(uint8_t nbvar){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, nbvar);
        };	

        int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadEncoderCountRelative(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadBrushlessCountRelative(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadBrushlessMotorSpeed(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int16_t ReadBatteryAmps(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
            std::cout << "Amps!" << std::endl;
        };	

        uint16_t ReadInternalVoltages(uint8_t param){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, param);
        };	

        uint32_t ReadAllDigitalInputs(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        int8_t ReadCaseAndInternalTemperatures(uint8_t param){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, param);
        };	

        int16_t ReadFeedback(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        uint8_t ReadStatusFlags(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        uint8_t ReadFaultFlags(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        uint8_t ReadCurrentDigitalOutputs(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        int32_t ReadClosedLoopError(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int32_t ReadUserBooleanVariable(uint8_t nbvar){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, nbvar);
        };	

        int32_t ReadInternalSerialCommand(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        uint32_t ReadTime(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        uint16_t ReadSpektrumRadioCapture(uint8_t nb_capture){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, nb_capture);
        };	

        uint8_t ReadDestinationPositionReachedFlag(uint8_t channel){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel);
        };	

        int32_t ReadMEMSAccelerometerAxis(uint8_t axis){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, axis);
        };	

        uint16_t ReadMagsensorTrackDetect(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        uint8_t ReadMagsensorTrackPosition(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        uint8_t ReadMagsensorMarkers(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        uint8_t ReadMagsensorStatus(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	
        
        uint8_t ReadMotorStatusFlags(void){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, 0);
        };	

        int32_t ReadIndividualDigitalInputs(uint8_t input){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, input);
        };	

        int16_t ReadAnalogInputs(uint8_t input){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, input);
        };	

        int16_t ReadAnalogInputsConverted(uint8_t input){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, input);
        };	
        
        int16_t ReadPulseInputs(uint8_t input){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, input);
        };	

        int16_t ReadPulseInputsConverted(uint8_t input){
            _comm->upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, input);
        };	

        // private :
        std::unique_ptr<i_comm> _comm;
        
    };

} //namespace roboteq 

