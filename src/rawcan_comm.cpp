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
        {runtime_command::SET_VELOCITY, 0x2002} } ;

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

} // namespace roboteq
