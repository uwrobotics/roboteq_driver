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

    const std::unordered_map<send_runtime_command, uint16_t> rawcan_comm::_runtime_command_map = { 
        {send_runtime_command::SET_MOTOR_COMMAND, 0x2000}, 
        {send_runtime_command::SET_POSITION, 0x2001}, 
        {send_runtime_command::SET_VELOCITY, 0x2002} 
    };

    const std::unordered_map<send_runtime_query, uint16_t> rawcan_comm::_runtime_query_map = { 
        {send_runtime_query::READ_MOTOR_AMPS, 0x2100}, 
        {send_runtime_query::READ_ACTUAL_MOTOR_COMMAND, 0x2101}, 
        {send_runtime_query::READ_ACTUAL_POWER_LEVEL, 0x2102} 
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

    bool rawcan_comm::download(send_runtime_command command, uint8_t subindex, uint32_t data) {

        struct can_frame frame;

        frame.can_id  = sdo_cob_id_offset + roboteq::rawcan_comm::_roboteq_can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x20;
        frame.data[1] = rawcan_comm::_runtime_command_map.at(command);
        frame.data[2] = rawcan_comm::_runtime_command_map.at(command) >> 8;
        frame.data[3] = subindex;
        frame.data[4] = data >> 0;
        frame.data[5] = data >> 8;
        frame.data[6] = data >> 16;
        frame.data[7] = data >> 24;

        write(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));

        return 1;
        
    }

    uint32_t rawcan_comm::upload(send_runtime_query query, uint8_t subindex) {
        
        struct can_frame frame;

        frame.can_id  = sdo_cob_id_offset + roboteq::rawcan_comm::_roboteq_can_id;
        frame.can_dlc = 8;
        frame.data[0] = 0x48;
        frame.data[1] = rawcan_comm::_runtime_query_map.at(query);
        frame.data[2] = rawcan_comm::_runtime_query_map.at(query) >> 8;
        frame.data[3] = subindex;
        frame.data[4] = 0 >> 0;
        frame.data[5] = 0 >> 8;
        frame.data[6] = 0 >> 16;
        frame.data[7] = 0 >> 24;

        write(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));
        _nbytes = read(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));

        return _nbytes;
        
    }    

} // namespace roboteq
