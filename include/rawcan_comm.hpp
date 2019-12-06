#pragma once

#include <linux/can.h>
#include <unordered_map>

#include "i_comm.hpp"


namespace roboteq { 

    class rawcan_comm : public i_comm {

        public:
        
        explicit rawcan_comm(canid_t roboteq_can_id = 0x1, std::string ifname = "can0");
        ~rawcan_comm(){}

        bool download(runtime_command command, uint8_t subindex, uint32_t data) override;
        uint32_t upload(runtime_query query, uint8_t subindex) override;

        private :
        int _roboteq_can_id;
        int _socket_handle;
        static const std::unordered_map<runtime_command, uint16_t> runtime_command_map;
        static const std::unordered_map<runtime_query, uint16_t> runtime_query_map;
    };

} // namespace roboteq