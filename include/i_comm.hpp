#pragma once

namespace roboteq {

    enum class send_runtime_command {

        SET_MOTOR_COMMAND =0,
        SET_POSITION,
        SET_VELOCITY

    };

    enum class send_runtime_query {

        READ_MOTOR_AMPS =0,
        READ_ACTUAL_MOTOR_COMMAND,
        READ_ACTUAL_POWER_LEVEL,

    };

    class i_comm {

        public :

        i_comm(){};
        virtual ~i_comm(){}

        virtual bool download(send_runtime_command command, uint8_t subindex, uint32_t data)=0;
        virtual uint32_t upload(send_runtime_query query, uint8_t subindex)=0;
                
        private :

    };

} //namespace roboteq