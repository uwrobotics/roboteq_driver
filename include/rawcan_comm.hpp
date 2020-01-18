#pragma once

#include <linux/can.h>
#include <unordered_map>

#include "i_comm.hpp"


namespace roboteq { 
    
     typedef struct command_properties {
        int canopen_index;
        int number_of_data_bytes;
        } command_properties_t;
    
    class rawcan_comm : public i_comm {

        public:
        
        explicit rawcan_comm(canid_t roboteq_can_id = 0x1, std::string ifname = "can0");
        ~rawcan_comm(){}

        bool download(send_runtime_command command, uint8_t subindex, uint32_t data) override;
        uint32_t upload(send_runtime_query query, uint8_t subindex) override;

        private :
        int _roboteq_can_id;
        int _socket_handle;
        uint32_t _nbytes;
        const uint16_t _sdo_command = 0x20;
        const uint16_t _sdo_query = 0x40;
        const uint16_t _sdo_cob_id_offset = 0x1;
        static const std::unordered_map<send_runtime_command, command_properties_t> _runtime_command_map;
        static const std::unordered_map<send_runtime_query, command_properties_t> _runtime_query_map;
    };

} // namespace roboteq