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
        int32_t SetPosition(int32_t position, uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_POSITION, channel, position));
            return sdo_response;
        };

        int16_t SetVelocity(int32_t speed, uint8_t channel){
           int16_t sdo_response = static_cast<int16_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_VELOCITY, channel, speed));
            return sdo_response;
        };
        	
        int32_t SetEncoderCounter(int32_t counter, uint8_t channel){
           int32_t sdo_response = static_cast<int32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_ENCODER_COUNTER, channel, counter));
            return sdo_response;
        };
        
        int32_t SetBrushlessCounter(int32_t counter, uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_BRUSHLESS_COUNTER, channel, counter));
            return sdo_response;
        };

        int32_t SetUserIntVariable(int32_t var, uint8_t nbvar){
           int32_t sdo_response = static_cast<int32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_USER_INT_VARIABLE, nbvar, var));
            return sdo_response;
        };

        int32_t SetAcceleration(int32_t accel, uint8_t channel){
           int32_t sdo_response = static_cast<int32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_ACCELERATION, channel, accel));
            return sdo_response;
        };

        int32_t SetDeceleration(int32_t decel, uint8_t channel){
           int32_t sdo_response = static_cast<int32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_DECELERATION, channel, decel));
            return sdo_response;
        };

        uint8_t SetAllDigitalOutBits(uint8_t out_bits){
           uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_ALL_DIGITAL_OUT_BITS, 0, out_bits));
        };

        uint8_t SetIndividualDigitalOutBits(uint8_t out_bits){
           uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0, out_bits));
            return sdo_response;
        };

        uint8_t ResetIndividualOutBits(uint8_t out_bits){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::RESET_INDIVIDUAL_OUT_BITS, 0, out_bits));
            return sdo_response;
        };

        uint8_t LoadHomeCounter(uint8_t channel){
           uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::LOAD_HOME_COUNTER, 0, 0));
            return sdo_response;
        };

        uint8_t EmergencyShutdown(void){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::EMERGENCY_SHUTDOWN, 0, 0));
            return sdo_response;
        };

        uint8_t ReleaseShutdown(void){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::RELEASE_SHUTDOWN, 0, 0));
            return sdo_response;
        };

        uint8_t StopInAllModes(uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::STOP_IN_ALL_MODES, channel, 0));
            return sdo_response;
        };

        uint8_t SetPosRelative(int32_t position, uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_POS_RELATIVE, channel, position));
            return sdo_response;
        };

        uint8_t SetNextPosAbsolute(int32_t position, uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_POS_ABSOLUTE, channel, position));
            return sdo_response;
        };

        uint8_t SetNextPosRelative(int32_t position, uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_POS_RELATIVE, channel, position));
            return sdo_response;
        };

        uint8_t SetNextAcceleration(int32_t accel, uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_ACCELERATION, channel, accel));
            return sdo_response;
        };

        uint8_t SetNextDeceleration(int32_t decel, uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_DECELERATION, channel, decel));
            return sdo_response;
        };

        uint8_t SetNextVelocity(int32_t speed, uint8_t channel){
           uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_NEXT_VELOCITY, channel, speed));
            return sdo_response;
        };

        uint32_t SetUserBoolVariable(uint32_t var, uint8_t nbvar){
            uint32_t sdo_response = static_cast<uint32_t>(_comm->sdo_download(roboteq::send_runtime_command::SET_USER_BOOL_VARIABLE, nbvar, var));
            return sdo_response;
        };

        uint8_t SaveConfigToFlash(void){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_download(roboteq::send_runtime_command::SAVE_CONFIG_TO_FLASH, 0, 0));
            return sdo_response;
        };	

        // read methods	
        int16_t ReadMotorAmps(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MOTOR_AMPS, channel));
            return sdo_response;
        };	

        int16_t ReadActualMotorCommand(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ACTUAL_MOTOR_COMMAND, channel));
            return sdo_response;
        };	

        int16_t ReadAppliedPowerLevel(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ACTUAL_POWER_LEVEL, channel));
            return sdo_response;
        };	

        int32_t ReadEncoderMotorSpeed(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_MOTOR_SPEED, channel));
            return sdo_response;
        };	
        
        int32_t ReadAbsoluteEncoderCount(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ABSOLUTE_ENCODER_COUNT, channel));
            return sdo_response;
        };	

        int32_t ReadAbsoluteBrushlessCounter(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ABSOLUTE_BRUSHLESS_COUNTER, channel));
            return sdo_response;
        };	

        int32_t ReadUserIntegerVariable(int32_t nbvar){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_USER_INTEGER_VARIABLE, nbvar));
            return sdo_response;
        };	

        int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
            return sdo_response;
        };	

        int32_t ReadEncoderCountRelative(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ENCODER_COUNT_RELATIVE, channel));
            return sdo_response;
        };	

        int32_t ReadBrushlessCountRelative(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_COUNT_RELATIVE, channel));
            return sdo_response;
        };	

        int16_t ReadBrushlessMotorSpeed(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED, channel));
            return sdo_response;
        };	

        int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, channel));
            return sdo_response;
        };	

        int16_t ReadBatteryAmps(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_BATTERY_AMPS, channel));
            return sdo_response;
        };	

        uint16_t ReadInternalVoltages(uint8_t param){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_VOLTAGES, param));
            return sdo_response;
        };	

        uint32_t ReadAllDigitalInputs(void){
            uint32_t sdo_response = static_cast<uint32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_ALL_DIGITAL_INPUTS, 0));
            return sdo_response;
        };	

        int8_t ReadCaseAndInternalTemperatures(uint8_t param){
            int8_t sdo_response = static_cast<int8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_CASE_AND_INTERNAL_TEMPERATURES, param));
            return sdo_response;
        };	

        int16_t ReadFeedback(uint8_t channel){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_FEEDBACK, channel));
            return sdo_response;
        };	

        uint16_t ReadStatusFlags(void){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_STATUS_FLAGS, 0));
            return sdo_response;
        };	

        uint16_t ReadFaultFlags(void){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_FAULT_FLAGS, 0));
            return sdo_response;
        };	

        uint16_t ReadCurrentDigitalOutputs(void){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_CURRENT_DIGITAL_OUTPUTS, 0));
            return sdo_response;
        };	

        int32_t ReadClosedLoopError(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_CLOSED_LOOP_ERROR, channel));
            return sdo_response;
        };	

        bool ReadUserBooleanVariable(uint32_t nbvar){
            bool sdo_response = static_cast<bool>(_comm->sdo_upload(roboteq::send_runtime_query::READ_USER_BOOLEAN_VARIABLE, nbvar));
            return sdo_response;
        };	

        int32_t ReadInternalSerialCommand(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_SERIAL_COMMAND, channel));
            return sdo_response;
        };

        int32_t ReadInternalAnalogCommand(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_ANALOG_COMMAND, channel));
            return sdo_response;
        };

        int32_t ReadInternalInternalPulseCommand(uint8_t channel){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_INTERNAL_PULSE_COMMAND, channel));
            return sdo_response;
        };	

        uint32_t ReadTime(uint8_t param){
            uint32_t sdo_response = static_cast<uint32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_TIME, param));
            return sdo_response;
        };	

        uint16_t ReadSpektrumRadioCapture(uint8_t nb_capture){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_SPEKTRUM_RADIO_CAPTURE, nb_capture));
            return sdo_response;
        };	

        uint8_t ReadDestinationPositionReachedFlag(uint8_t channel){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_DESTINATION_POSITION_REACHED_FLAG, channel));
            return sdo_response;
        };	

        int32_t ReadMEMSAccelerometerAxis(uint8_t axis){
            int32_t sdo_response = static_cast<int32_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MEMS_ACCELEROMETER_AXIS, axis));
            return sdo_response;
        };	

        uint8_t ReadMagsensorTrackDetect(void){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_TRACK_DETECT, 0));
            return sdo_response;
        };	

        int16_t ReadMagsensorTrackPosition(uint8_t nb_pulse){
            int16_t sdo_response = static_cast<int16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_TRACK_POSITION, nb_pulse));
            return sdo_response;
        };	

        uint8_t ReadMagsensorMarkers(uint8_t nb_pulse){
            uint8_t sdo_response = static_cast<uint8_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_MARKERS, nb_pulse));
            return sdo_response;
        };	

        uint16_t ReadMagsensorStatus(uint8_t nb_pulse){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MAGSENSOR_STATUS, nb_pulse));
            return sdo_response;
        };	
        
        uint16_t ReadMotorStatusFlags(uint8_t nb_pulse){
            uint16_t sdo_response = static_cast<uint16_t>(_comm->sdo_upload(roboteq::send_runtime_query::READ_MOTOR_STATUS_FLAGS, nb_pulse));
            return sdo_response;
        };

        // private :
        std::unique_ptr<i_comm> _comm;
        
    };

} //namespace roboteq 

