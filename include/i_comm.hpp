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

        virtual bool download(runtime_command command, uint8_t argument1, uint32_t argument2)=0;
        virtual uint32_t upload(runtime_query command, uint8_t argument1)=0;
                
        private :

    };

} //namespace roboteq