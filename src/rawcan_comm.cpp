#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <iostream>
#include <cstring>

#include "rawcan_comm.hpp"

namespace roboteq { 

    const std::unordered_map<runtime_command, uint16_t> rawcan_comm::runtime_command_map = { 
        {runtime_command::SET_MOTOR_COMMAND, 0x2000}, 
        {runtime_command::SET_POSITION, 0x2001}, 
        {runtime_command::SET_VELOCITY, 0x2002} 
    };

    const std::unordered_map<runtime_query, uint16_t> rawcan_comm::runtime_query_map = { 
        {runtime_query::READ_MOTOR_AMPS, 0x2100}, 
        {runtime_query::READ_ACTUAL_MOTOR_COMMAND, 0x2101}, 
        {runtime_query::READ_ACTUAL_POWER_LEVEL, 0x2102} 
    };

    rawcan_comm::rawcan_comm(canid_t roboteq_can_id, std::string ifname): _roboteq_can_id(roboteq_can_id) {

        if((_socket_handle = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            printf("Error while opening socket");
            throw -1;
        }

        struct ifreq ifr;

        std::strcpy(ifr.ifr_name, ifname.c_str());
        ioctl(_socket_handle, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

        if(bind(_socket_handle, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            printf("Error in socket bind");
            throw -2;
        }
    }

    bool rawcan_comm::download(runtime_command command, uint8_t subindex, uint32_t data) {

        struct can_frame frame;

        if (rawcan_comm::runtime_command_map.find(command) == rawcan_comm::runtime_command_map.end()) {
                printf("Error in command");
            throw -1; 
            }
        
        std::unordered_map<runtime_command, uint16_t>::const_iterator command_map = rawcan_comm::runtime_command_map.find(command);

        frame.can_id  = 0x600 + roboteq::rawcan_comm::_roboteq_can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x20;
        frame.data[1] = command_map->second;
        frame.data[2] = command_map->second >> 8;
        frame.data[3] = subindex;
        frame.data[4] = data >> 0;
        frame.data[5] = data >> 8;
        frame.data[6] = data >> 16;
        frame.data[7] = data >> 24;

        write(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));

        return 1;
        
    }

    uint32_t rawcan_comm::upload(runtime_query query, uint8_t subindex) {
        
        struct can_frame frame;

        if (rawcan_comm::runtime_query_map.find(query) == rawcan_comm::runtime_query_map.end()) {
                printf("Error in query");
            throw -1; 
            }
        
        std::unordered_map<runtime_query, uint16_t>::const_iterator query_map = rawcan_comm::runtime_query_map.find(query);

        frame.can_id  = 0x600 + roboteq::rawcan_comm::_roboteq_can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x48;
        frame.data[1] = query_map->second;
        frame.data[2] = query_map->second >> 8;
        frame.data[3] = subindex;
        frame.data[4] = 0 >> 0;
        frame.data[5] = 0 >> 8;
        frame.data[6] = 0 >> 16;
        frame.data[7] = 0 >> 24;

        write(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));
        read(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));

        return 1;
        
    }    

} // namespace roboteq
