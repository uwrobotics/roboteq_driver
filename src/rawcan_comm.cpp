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

#include "roboteq_controller/rawcan_comm.hpp"

namespace roboteq { 

    void rawcan_comm::rawcan_comm(canid_t roboteq_can_id = 1, std::string ifname = "can0"): _roboteq_can_id(roboteq_can_id){

        if((_socket_handle = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            printf("Error while opening socket");
            throw -1;
        }

        struct ifreq ifr;

        strcpy(ifr.ifr_name, ifname);
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