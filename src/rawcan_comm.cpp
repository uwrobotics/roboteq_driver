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

    const std::unordered_map<send_runtime_command, command_properties_t> rawcan_comm::_runtime_command_map = { 
  
        {send_runtime_command::SET_POSITION, {0x2001, 0x0}}, 
        {send_runtime_command::SET_VELOCITY, {0x2002, 0x2}},
        {send_runtime_command::SET_ENCODER_COUNTER, {0x2003, 0x0}},
        {send_runtime_command::SET_BRUSHLESS_COUNTER, {0x2004, 0x0}},
        {send_runtime_command::SET_USER_INT_VARIABLE, {0x2005, 0x0}},
        {send_runtime_command::SET_ACCELERATION, {0x2006, 0x0}},
        {send_runtime_command::SET_DECELERATION, {0x2007, 0x0}},
        {send_runtime_command::SET_ALL_DIGITAL_OUT_BITS, {0x2008, 0x3}},
        {send_runtime_command::SET_INDIVIDUAL_DIGITAL_OUT_BITS, {0x2009, 0x3}},
        {send_runtime_command::RESET_INDIVIDUAL_OUT_BITS, {0x200a, 0x3}},
        {send_runtime_command::LOAD_HOME_COUNTER, {0x200b, 0x3}},
        {send_runtime_command::EMERGENCY_SHUTDOWN, {0x200c, 0x3}},
        {send_runtime_command::RELEASE_SHUTDOWN, {0x200d, 0x3}},
        {send_runtime_command::STOP_IN_ALL_MODES, {0x200e, 0x3}},
        {send_runtime_command::SET_POS_RELATIVE, {0x200f, 0x0}},
        {send_runtime_command::SET_NEXT_POS_ABSOLUTE, {0x2010, 0x0}},
        {send_runtime_command::SET_NEXT_POS_RELATIVE, {0x2011, 0x0}},
        {send_runtime_command::SET_NEXT_ACCELERATION, {0x2012, 0x0}},
        {send_runtime_command::SET_NEXT_DECELERATION, {0x2013, 0x0}},
        {send_runtime_command::SET_NEXT_VELOCITY, {0x2014, 0x2}},
        {send_runtime_command::SET_USER_BOOL_VARIABLE, {0x2015, 0x0}},
        {send_runtime_command::SAVE_CONFIG_TO_FLASH, {0x2017, 0x4}},
    };

    const std::unordered_map<send_runtime_query, command_properties_t> rawcan_comm::_runtime_query_map = { 
        {send_runtime_query::READ_MOTOR_AMPS, {0x2100, 0x3}}, 
        {send_runtime_query::READ_ACTUAL_MOTOR_COMMAND, {0x2101, 0x3}}, 
        {send_runtime_query::READ_ACTUAL_POWER_LEVEL, {0x2102, 0x3}}, 
        {send_runtime_query::READ_ENCODER_MOTOR_SPEED, {0x2103, 0x3}}, 
        {send_runtime_query::READ_ABSOLUTE_ENCODER_COUNT, {0x2104, 0x3}}, 
        {send_runtime_query::READ_ABSOLUTE_BRUSHLESS_COUNTER, {0x2105, 0x3}}, 
        {send_runtime_query::READ_USER_INTEGER_VARIABLE, {0x2106, 0x3}}, 
        {send_runtime_query::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, {0x2107, 0x3}}, 
        {send_runtime_query::READ_ENCODER_COUNT_RELATIVE, {0x2108, 0x3}}, 
        {send_runtime_query::READ_BRUSHLESS_COUNT_RELATIVE, {0x2109, 0x3}},
        {send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED, {0x210a, 0x3}}, 
        {send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, {0x210b, 0x3}}, 
        {send_runtime_query::READ_BATTERY_AMPS, {0x210c, 0x3}}, 
        {send_runtime_query::READ_INTERNAL_VOLTAGES, {0x210d, 0x3}}, 
        {send_runtime_query::READ_ALL_DIGITAL_INPUTS, {0x210e, 0x0}}, 
        {send_runtime_query::READ_CASE_AND_INTERNAL_TEMPERATURES, {0x210f, 0x3}}, 
        {send_runtime_query::READ_FEEDBACK, {0x2110, 0x3}},
        {send_runtime_query::READ_STATUS_FLAGS, {0x211, 0x0}}, 
        {send_runtime_query::READ_FAULT_FLAGS, {0x2112, 0x0}}, 
        {send_runtime_query::READ_CURRENT_DIGITAL_OUTPUTS, {0x2113, 0x0}}, 
        {send_runtime_query::READ_CLOSED_LOOP_ERROR, {0x2114, 0x3}}, 
        {send_runtime_query::READ_USER_BOOLEAN_VARIABLE, {0x2115, 0x3}}, 
        {send_runtime_query::READ_INTERNAL_SERIAL_COMMAND, {0x2116, 0x3}}, 
        {send_runtime_query::READ_INTERNAL_ANALOG_COMMAND, {0x2117, 0x3}},
        {send_runtime_query::READ_INTERNAL_PULSE_COMMAND, {0x2118, 0x3}}, 
        {send_runtime_query::READ_TIME, {0x2119, 0x0}}, 
        {send_runtime_query::READ_SPEKTRUM_RADIO_CAPTURE, {0x211a, 0x3}}, 
        {send_runtime_query::READ_DESTINATION_POSITION_REACHED_FLAG, {0x211b, 0x3}}, 
        {send_runtime_query::READ_MEMS_ACCELEROMETER_AXIS, {0x211c, 0x3}}, 
        {send_runtime_query::READ_MAGSENSOR_TRACK_DETECT, {0x211d, 0x0}}, 
        {send_runtime_query::READ_MAGSENSOR_TRACK_POSITION, {0x211e, 0x0}},
        {send_runtime_query::READ_MAGSENSOR_MARKERS, {0x211f, 0x0}}, 
        {send_runtime_query::READ_MAGSENSOR_STATUS, {0x2120, 0x0}}, 
        {send_runtime_query::READ_MOTOR_STATUS_FLAGS, {0x2121, 0x0}}, 
        {send_runtime_query::READ_INDIVIDUAL_DIGITAL_INPUTS, {0x6400, 0x3}}, 
        {send_runtime_query::READ_ANALOG_INPUTS, {0x6401, 0x3}}, 
        {send_runtime_query::READ_ANALOG_INPUTS_CONVERTED, {0x6402, 0x3}}, 
        {send_runtime_query::READ_PULSE_INPUTS, {0x6403, 0x3}},
        {send_runtime_query::READ_PULSE_INPUTS_CONVERTED, {0x6404, 0x3}},
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

        std::cout << ifname << " at index " << ifr.ifr_ifindex << std::endl;

        if(bind(_socket_handle, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            printf("Error in socket bind");
            throw -2;
        }

        struct can_filter rfilter[4];

        rfilter[0].can_id   = 0x001;
        rfilter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
        rfilter[1].can_id   = 0x002;
        rfilter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
        rfilter[2].can_id   = 0x003;
        rfilter[2].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
        rfilter[3].can_id   = 0x004;
        rfilter[3].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

        setsockopt(_socket_handle, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        int join_filter = 1;
        setsockopt(_socket_handle, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join_filter, sizeof(join_filter));
    
    }

    bool rawcan_comm::download(send_runtime_command command, uint8_t subindex, uint32_t data) {

        struct can_frame frame;

        frame.can_id  = _sdo_cob_id_offset + roboteq::rawcan_comm::_roboteq_can_id;
        frame.can_dlc = 8;
        frame.data[0] = _sdo_command + rawcan_comm::_runtime_command_map.at(command).number_of_data_bytes;
        frame.data[1] = rawcan_comm::_runtime_command_map.at(command).canopen_index; 
        frame.data[2] = rawcan_comm::_runtime_command_map.at(command).canopen_index >> 8;
        frame.data[3] = subindex;
        frame.data[4] = data >> 0;
        frame.data[5] = data >> 8;
        frame.data[6] = data >> 16;
        frame.data[7] = data >> 24;

        std::cout <<  "Write" << std::endl;
            
        write(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));
        _nbytes = read(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));

        std::cout <<  "Wrote" << std::endl;

        if (_nbytes == sizeof(struct can_frame)) {
                printf("Invalid CAN frame\n");
                }
        else {
                return 0;
                }
    }

    uint32_t rawcan_comm::upload(send_runtime_query query, uint8_t subindex) {
        
        struct can_frame frame;

        frame.can_id  = _sdo_cob_id_offset + roboteq::rawcan_comm::_roboteq_can_id;
        frame.can_dlc = 8;
         frame.data[0] = _sdo_query + rawcan_comm::_runtime_query_map.at(query).number_of_data_bytes;
        frame.data[1] = rawcan_comm::_runtime_query_map.at(query).canopen_index; 
        frame.data[2] = rawcan_comm::_runtime_query_map.at(query).canopen_index >> 8;
        frame.data[3] = subindex;
        frame.data[4] = 0 >> 0;
        frame.data[5] = 0 >> 8;
        frame.data[6] = 0 >> 16;
        frame.data[7] = 0 >> 24;

        std::cout <<  "Write" << std::endl;

        write(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));
        _nbytes = read(roboteq::rawcan_comm::_socket_handle, &frame, sizeof(struct can_frame));

        std::cout <<  "Wrote" << std::endl;

        if (_nbytes != sizeof(struct can_frame)) {
                printf("Invalid CAN frame\n");
                }
        else {
                return _nbytes;
                }
        
    }    

} // namespace roboteq
