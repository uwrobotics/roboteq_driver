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

#include "canopen_comm.hpp"

namespace roboteq {  

    const std::unordered_map<send_runtime_command, command_properties_t> canopen_comm::_runtime_command_map = { 
  
        {send_runtime_command::SET_POSITION, {0x2001, 0}}, 
        {send_runtime_command::SET_VELOCITY, {0x2002, 2}},
        {send_runtime_command::SET_ENCODER_COUNTER, {0x2003, 0}},
        {send_runtime_command::SET_BRUSHLESS_COUNTER, {0x2004, 0}},
        {send_runtime_command::SET_USER_INT_VARIABLE, {0x2005, 0}},
        {send_runtime_command::SET_ACCELERATION, {0x2006, 0}},
        {send_runtime_command::SET_DECELERATION, {0x2007, 0}},
        {send_runtime_command::SET_ALL_DIGITAL_OUT_BITS, {0x2008, 3}},
        {send_runtime_command::SET_INDIVIDUAL_DIGITAL_OUT_BITS, {0x2009, 3}},
        {send_runtime_command::RESET_INDIVIDUAL_OUT_BITS, {0x200a, 3}},
        {send_runtime_command::LOAD_HOME_COUNTER, {0x200b, 3}},
        {send_runtime_command::EMERGENCY_SHUTDOWN, {0x200c, 3}},
        {send_runtime_command::RELEASE_SHUTDOWN, {0x200d, 3}},
        {send_runtime_command::STOP_IN_ALL_MODES, {0x200e, 3}},
        {send_runtime_command::SET_POS_RELATIVE, {0x200f, 3}},
        {send_runtime_command::SET_NEXT_POS_ABSOLUTE, {0x2010, 3}},
        {send_runtime_command::SET_NEXT_POS_RELATIVE, {0x2011, 3}},
        {send_runtime_command::SET_NEXT_ACCELERATION, {0x2012, 3}},
        {send_runtime_command::SET_NEXT_DECELERATION, {0x2013, 3}},
        {send_runtime_command::SET_NEXT_VELOCITY, {0x2014, 3}},
        {send_runtime_command::SET_USER_BOOL_VARIABLE, {0x2015, 0}},
        {send_runtime_command::SAVE_CONFIG_TO_FLASH, {0x2017, 3}},
    };

    const std::unordered_map<send_runtime_query, command_properties_t> canopen_comm::_runtime_query_map = { 
        {send_runtime_query::READ_MOTOR_AMPS, {0x2100, 2}}, 
        {send_runtime_query::READ_ACTUAL_MOTOR_COMMAND, {0x2101, 2}}, 
        {send_runtime_query::READ_ACTUAL_POWER_LEVEL, {0x2102, 2}}, 
        {send_runtime_query::READ_ENCODER_MOTOR_SPEED, {0x2103, 0}}, 
        {send_runtime_query::READ_ABSOLUTE_ENCODER_COUNT, {0x2104, 0}}, 
        {send_runtime_query::READ_ABSOLUTE_BRUSHLESS_COUNTER, {0x2105, 0}}, 
        {send_runtime_query::READ_USER_INTEGER_VARIABLE, {0x2106, 0}}, 
        {send_runtime_query::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, {0x2107, 2}}, 
        {send_runtime_query::READ_ENCODER_COUNT_RELATIVE, {0x2108, 0}}, 
        {send_runtime_query::READ_BRUSHLESS_COUNT_RELATIVE, {0x2109, 0}},
        {send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED, {0x210a, 2}}, 
        {send_runtime_query::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, {0x210b, 2}}, 
        {send_runtime_query::READ_BATTERY_AMPS, {0x210c, 2}}, 
        {send_runtime_query::READ_INTERNAL_VOLTAGES, {0x210D, 2}}, 
        {send_runtime_query::READ_ALL_DIGITAL_INPUTS, {0x210e, 0}}, 
        {send_runtime_query::READ_CASE_AND_INTERNAL_TEMPERATURES, {0x210f, 3}}, 
        {send_runtime_query::READ_FEEDBACK, {0x2110, 2}},
        {send_runtime_query::READ_STATUS_FLAGS, {0x2111, 2}}, 
        {send_runtime_query::READ_FAULT_FLAGS, {0x2112, 2}}, 
        {send_runtime_query::READ_CURRENT_DIGITAL_OUTPUTS, {0x2113, 2}}, 
        {send_runtime_query::READ_CLOSED_LOOP_ERROR, {0x2114, 0}}, 
        {send_runtime_query::READ_USER_BOOLEAN_VARIABLE, {0x2115, 0}}, 
        {send_runtime_query::READ_INTERNAL_SERIAL_COMMAND, {0x2116, 0}}, 
        {send_runtime_query::READ_INTERNAL_ANALOG_COMMAND, {0x2117, 0}},
        {send_runtime_query::READ_INTERNAL_PULSE_COMMAND, {0x2118, 0}}, 
        {send_runtime_query::READ_TIME, {0x2119, 0}}, 
        {send_runtime_query::READ_SPEKTRUM_RADIO_CAPTURE, {0x211a, 2}}, 
        {send_runtime_query::READ_DESTINATION_POSITION_REACHED_FLAG, {0x211b, 3}}, 
        {send_runtime_query::READ_MEMS_ACCELEROMETER_AXIS, {0x211c, 0}}, 
        {send_runtime_query::READ_MAGSENSOR_TRACK_DETECT, {0x211d, 3}}, 
        {send_runtime_query::READ_MAGSENSOR_TRACK_POSITION, {0x211e, 2}},
        {send_runtime_query::READ_MAGSENSOR_MARKERS, {0x211f, 3}}, 
        {send_runtime_query::READ_MAGSENSOR_STATUS, {0x2120, 2}}, 
        {send_runtime_query::READ_MOTOR_STATUS_FLAGS, {0x2121, 2}}, 
    };

    canopen_comm::canopen_comm(canid_t roboteq_can_id, std::string ifname): _roboteq_can_id(roboteq_can_id) {

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

        struct can_filter rfilter[2];

        rfilter[0].can_id   = _sdo_cob_id_offset + roboteq::canopen_comm::_roboteq_can_id;
        rfilter[0].can_mask = CAN_SFF_MASK;
        rfilter[1].can_id   = _sdo_response_cob_id_offset + roboteq::canopen_comm::_roboteq_can_id;
        rfilter[1].can_mask = CAN_SFF_MASK;

        setsockopt(_socket_handle, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        if(bind(_socket_handle, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            printf("Error in socket bind");
            throw -2;
        }
    
    }

    bool canopen_comm::sdo_download(send_runtime_command command, uint8_t subindex, uint32_t data) {

        struct can_frame frame;

        frame.can_id  = _sdo_cob_id_offset + roboteq::canopen_comm::_roboteq_can_id;
        frame.can_dlc = 8;
        frame.data[0] = (_sdo_command<<4) | (canopen_comm::_runtime_command_map.at(command).number_of_unused_bytes<<2);
        frame.data[1] = canopen_comm::_runtime_command_map.at(command).canopen_index; 
        frame.data[2] = canopen_comm::_runtime_command_map.at(command).canopen_index >> 8;
        frame.data[3] = subindex;
        frame.data[4] = data >> 0;
        frame.data[5] = data >> 8;
        frame.data[6] = data >> 16;
        frame.data[7] = data >> 24;

        write(roboteq::canopen_comm::_socket_handle, &frame, sizeof(struct can_frame));

        struct can_frame response_frame = {};

        uint32_t nbytes = read(roboteq::canopen_comm::_socket_handle, &response_frame, sizeof(struct can_frame));

        std::cout <<  std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t" << static_cast<unsigned>(response_frame.data[0])
                                << "\t" << static_cast<unsigned>(response_frame.data[1]) << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t" << static_cast<unsigned>(response_frame.data[3]) 
                                << "\t" << static_cast<unsigned>(response_frame.data[4]) << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t" << static_cast<unsigned>(response_frame.data[6])
                                << "\t" << static_cast<unsigned>(response_frame.data[7]) << std::endl;
    
        if (nbytes != sizeof(struct can_frame)) {
                printf("Invalid CAN frame\n");
                return 0;
                }
        else {
                return (response_frame.data[4] | response_frame.data[5] << 8 | response_frame.data[6] << 16 | response_frame.data[7] << 24);
                }
    }

    uint32_t canopen_comm::sdo_upload(send_runtime_query query, uint8_t subindex) {
        
        struct can_frame query_frame;

        query_frame.can_id  = _sdo_cob_id_offset + roboteq::canopen_comm::_roboteq_can_id;
        query_frame.can_dlc = 8;
        query_frame.data[0] = (_sdo_query<<4) | (canopen_comm::_runtime_query_map.at(query).number_of_unused_bytes<<2);
        query_frame.data[1] = canopen_comm::_runtime_query_map.at(query).canopen_index; 
        query_frame.data[2] = canopen_comm::_runtime_query_map.at(query).canopen_index >> 8;
        query_frame.data[3] = subindex;
        query_frame.data[4] = 0;
        query_frame.data[5] = 0;
        query_frame.data[6] = 0;
        query_frame.data[7] = 0;

        write(roboteq::canopen_comm::_socket_handle, &query_frame, sizeof(struct can_frame));

        struct can_frame response_frame = {};

        uint32_t nbytes = read(roboteq::canopen_comm::_socket_handle, &response_frame, sizeof(struct can_frame));

        std::cout <<  std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t" << static_cast<unsigned>(response_frame.data[0])
                                << "\t" << static_cast<unsigned>(response_frame.data[1]) << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t" << static_cast<unsigned>(response_frame.data[3]) 
                                << "\t" << static_cast<unsigned>(response_frame.data[4]) << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t" << static_cast<unsigned>(response_frame.data[6])
                                << "\t" << static_cast<unsigned>(response_frame.data[7]) << std::endl;
    
        if (nbytes != sizeof(struct can_frame)) {
                printf("Invalid CAN frame\n");
                return 0;
                }
        else {
                return (response_frame.data[4] | response_frame.data[5] << 8 | response_frame.data[6] << 16 | response_frame.data[7] << 24);
                }
        
    }    

} // namespace roboteq
