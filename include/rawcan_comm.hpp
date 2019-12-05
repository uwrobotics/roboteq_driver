#pragma once

#include <linux/can.h>
#include <unordered_map>

#include "i_comm.hpp"


namespace roboteq { 

    class rawcan_comm : public i_comm {

        public:
        
        explicit rawcan_comm(canid_t roboteq_can_id = 0x1, std::string ifname = "can0");
        ~rawcan_comm(){}

        // int Send_Initiate_SDO_Download(short index, short subindex, long data);
        bool write(runtime_command command, uint8_t argument1, uint32_t argument2) override;
        uint32_t read(runtime_query command, uint8_t argument1) override;

        private :
        int _roboteq_can_id;
        int _socket_handle;
        static const std::unordered_map<runtime_command, uint16_t> runtime_command_map;
    };

} // namespace roboteq