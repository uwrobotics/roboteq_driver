#pragma once

#include <linux/can.h>
#include "i_comm.hpp"

namespace roboteq { 

    class rawcan_comm : public i_comm {

        public:
        
        explicit rawcan_comm(canid_t _roboteq_can_id, std::string _ifname);
        ~rawcan_comm(){}

        // set methods
        void SetParameters(int16_t _velocity, long _accel, long _decel, uint8_t _channel);
        void SetPosition(long _position, uint8_t _channel);
        void SetVelocity(uint16_t _speed, uint8_t _channel);
        void SetEncoderCounter(long _counter, uint8_t _channel);
        void SetBrushlessCounter(long _counter, uint8_t _channel);
        void SetUserIntVariable(long _var, uint8_t _nb_var);
        void SetAcceleration(long _accel, uint8_t _channel);
        void SetDeceleration(long _decel, uint8_t _channel);
        void SetAllDigitalOutBits(uint8_t _out_bits);
        void SetIndividualDigitalOutBits(uint8_t _out_bits);
        void ResetIndividualOutBits(uint8_t _out_bits);
        void LoadHomeCounter(uint8_t _channel);
        void EmergencyShutdown(uint8_t _param);
        void ReleaseShutdown(uint8_t _param);
        void StopInAllModes(uint8_t _channel);
        void SetPosRelative(long _position, uint8_t _channel);
        void SetNextPosAbsolute(long _position, uint8_t _channel);
        void SetNextPosRelative(long _position, uint8_t _channel);
        void SetNextAcceleration(long _accel, uint8_t _channel);
        void SetNextDeceleration(long _decel, uint8_t _channel);
        void SetNextVelocity(uint16_t _speed, uint8_t _channel);
        void SetUserBoolVariable(long _var, uint8_t _nb_var);
        void SaveConfigToFlash(void);

        // read methods
        int16_t ReadMotorAmps(uint8_t _channel);
        int16_t ReadActualMotorCommand(uint8_t _channel);
        int16_t ReadAppliedPowerLevel(uint8_t _channel);
        int16_t ReadEncoderMotorSpeed(uint8_t _channel);
        int16_t ReadAbsoluteEncoderCount(uint8_t _channel);
        int16_t ReadAbsoluteBrushlessCounter(uint8_t _channel);
        int16_t ReadUserIntegerVariable(uint8_t _nb_var);
        int16_t ReadEncoderMotorSpeedRelativeToMaxSpeed(uint8_t _channel);
        int16_t ReadEncoderCountRelative(uint8_t _channel);
        int16_t ReadBrushlessCountRelative(uint8_t _channel);
        int16_t ReadBrushlessMotorSpeed(uint8_t _channel);
        int16_t ReadBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t _channel);
        int16_t ReadBatteryAmps(uint8_t _channel);
        uint16_t ReadInternalVoltages(uint8_t _param);
        uint32_t ReadAllDigitalInputs(void);
        int8_t ReadCaseAndInternalTemperatures(uint8_t _param);
        int16_t ReadFeedback(uint8_t _channel);
        uint8_t ReadStatusFlags(void);
        uint8_t ReadFaultFlags(void);
        uint8_t ReadCurrentDigitalOutputs(void);
        int32_t ReadClosedLoopError(uint8_t _channel);
        int32_t ReadUserBooleanVariable(uint8_t _nb_var);
        int32_t ReadInternalSerialCommand(uint8_t _channel);
        uint32_t ReadTime(void);
        uint16_t ReadSpektrumRadioCapture(uint8_t _nb_capture);
        uint8_t ReadDestinationPositionReachedFlag(uint8_t _channel);
        int32_t ReadMEMSAccelerometerAxis(uint8_t _axis);
        uint16_t ReadMagsensorTrackDetect(void);
        uint8_t ReadMagsensorTrackPosition(void);
        uint8_t ReadMagsensorMarkers(void);
        uint8_t ReadMagsensorStatus(void);
        uint8_t ReadMotorStatusFlags(void);
        int32_t ReadIndividualDigitalInputs(uint8_t _input);
        int16_t ReadAnalogInputs(uint8_t _input);
        int16_t ReadAnalogInputsConverted(uint8_t _input);
        int16_t ReadPulseInputs(uint8_t _input);
        int16_t ReadPulseInputsConverted(uint8_t _input);

        private :
        int _roboteq_can_id;
        int _socket_handle;

    };

} // namespace roboteq