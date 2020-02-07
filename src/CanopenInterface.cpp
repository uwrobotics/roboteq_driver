#include "CanopenInterface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>

namespace roboteq {

const std::unordered_map<RuntimeCommand, command_properties_t> CanopenInterface::RUNTIME_COMMAND_MAP_ = {

    {RuntimeCommand::SET_MOTOR_COMMAND, {0x2000, 0}},
    {RuntimeCommand::SET_POSITION, {0x2001, 0}},
    {RuntimeCommand::SET_VELOCITY, {0x2002, 2}},
    {RuntimeCommand::SET_ENCODER_COUNTER, {0x2003, 0}},
    {RuntimeCommand::SET_BRUSHLESS_COUNTER, {0x2004, 0}},
    {RuntimeCommand::SET_USER_INT_VARIABLE, {0x2005, 0}},
    {RuntimeCommand::SET_ACCELERATION, {0x2006, 0}},
    {RuntimeCommand::SET_DECELERATION, {0x2007, 0}},
    {RuntimeCommand::SET_ALL_DIGITAL_OUT_BITS, {0x2008, 3}},
    {RuntimeCommand::SET_INDIVIDUAL_DIGITAL_OUT_BITS, {0x2009, 3}},
    {RuntimeCommand::RESET_INDIVIDUAL_OUT_BITS, {0x200a, 3}},
    {RuntimeCommand::LOAD_HOME_COUNTER, {0x200b, 3}},
    {RuntimeCommand::EMERGENCY_SHUTDOWN, {0x200c, 4}},
    {RuntimeCommand::RELEASE_SHUTDOWN, {0x200d, 3}},
    {RuntimeCommand::STOP_IN_ALL_MODES, {0x200e, 3}},
    {RuntimeCommand::SET_POS_RELATIVE, {0x200f, 3}},
    {RuntimeCommand::SET_NEXT_POS_ABSOLUTE, {0x2010, 3}},
    {RuntimeCommand::SET_NEXT_POS_RELATIVE, {0x2011, 3}},
    {RuntimeCommand::SET_NEXT_ACCELERATION, {0x2012, 3}},
    {RuntimeCommand::SET_NEXT_DECELERATION, {0x2013, 3}},
    {RuntimeCommand::SET_NEXT_VELOCITY, {0x2014, 3}},
    {RuntimeCommand::SET_USER_BOOL_VARIABLE, {0x2015, 0}},
    {RuntimeCommand::SAVE_CONFIG_TO_FLASH, {0x2017, 3}},
};

const std::unordered_map<RuntimeQuery, command_properties_t> CanopenInterface::RUNTINE_QUERY_MAP_ = {
    {RuntimeQuery::READ_MOTOR_AMPS, {0x2100, 2}},
    {RuntimeQuery::READ_ACTUAL_MOTOR_COMMAND, {0x2101, 2}},
    {RuntimeQuery::READ_ACTUAL_POWER_LEVEL, {0x2102, 2}},
    {RuntimeQuery::READ_ENCODER_MOTOR_SPEED, {0x2103, 0}},
    {RuntimeQuery::READ_ABSOLUTE_ENCODER_COUNT, {0x2104, 0}},
    {RuntimeQuery::READ_ABSOLUTE_BRUSHLESS_COUNTER, {0x2105, 0}},
    {RuntimeQuery::READ_USER_INTEGER_VARIABLE, {0x2106, 0}},
    {RuntimeQuery::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, {0x2107, 2}},
    {RuntimeQuery::READ_ENCODER_COUNT_RELATIVE, {0x2108, 0}},
    {RuntimeQuery::READ_BRUSHLESS_COUNT_RELATIVE, {0x2109, 0}},
    {RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED, {0x210a, 2}},
    {RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, {0x210b, 2}},
    {RuntimeQuery::READ_BATTERY_AMPS, {0x210c, 2}},
    {RuntimeQuery::READ_INTERNAL_VOLTAGES, {0x210D, 2}},
    {RuntimeQuery::READ_ALL_DIGITAL_INPUTS, {0x210e, 0}},
    {RuntimeQuery::READ_CASE_AND_INTERNAL_TEMPERATURES, {0x210f, 3}},
    {RuntimeQuery::READ_FEEDBACK, {0x2110, 2}},
    {RuntimeQuery::READ_STATUS_FLAGS, {0x2111, 2}},
    {RuntimeQuery::READ_FAULT_FLAGS, {0x2112, 2}},
    {RuntimeQuery::READ_CURRENT_DIGITAL_OUTPUTS, {0x2113, 2}},
    {RuntimeQuery::READ_CLOSED_LOOP_ERROR, {0x2114, 0}},
    {RuntimeQuery::READ_USER_BOOLEAN_VARIABLE, {0x2115, 0}},
    {RuntimeQuery::READ_INTERNAL_SERIAL_COMMAND, {0x2116, 0}},
    {RuntimeQuery::READ_INTERNAL_ANALOG_COMMAND, {0x2117, 0}},
    {RuntimeQuery::READ_INTERNAL_PULSE_COMMAND, {0x2118, 0}},
    {RuntimeQuery::READ_TIME, {0x2119, 0}},
    {RuntimeQuery::READ_SPEKTRUM_RADIO_CAPTURE, {0x211a, 2}},
    {RuntimeQuery::READ_DESTINATION_POSITION_REACHED_FLAG, {0x211b, 3}},
    {RuntimeQuery::READ_MEMS_ACCELEROMETER_AXIS, {0x211c, 0}},
    {RuntimeQuery::READ_MAGSENSOR_TRACK_DETECT, {0x211d, 3}},
    {RuntimeQuery::READ_MAGSENSOR_TRACK_POSITION, {0x211e, 2}},
    {RuntimeQuery::READ_MAGSENSOR_MARKERS, {0x211f, 3}},
    {RuntimeQuery::READ_MAGSENSOR_STATUS, {0x2120, 2}},
    {RuntimeQuery::READ_MOTOR_STATUS_FLAGS, {0x2121, 2}},
};

CanopenInterface::CanopenInterface(canid_t roboteq_can_id, const std::string& ifname)
    : roboteq_can_id_(roboteq_can_id) {
  if ((socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    printf("Error while opening socket");
    throw - 1;
  }

  struct ifreq ifr {};

  std::strcpy(ifr.ifr_name, ifname.c_str());
  ioctl(socket_handle_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr {};

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  std::cout << ifname << " at index " << ifr.ifr_ifindex << std::endl;

  std::array<struct can_filter, 2> can_receive_filter{};

  can_receive_filter[0].can_id = sdo_cob_id_offset_ + roboteq::CanopenInterface::roboteq_can_id_;
  can_receive_filter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
  can_receive_filter[1].can_id = sdo_response_cob_id_offset_ + roboteq::CanopenInterface::roboteq_can_id_;
  can_receive_filter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);

  // struct timeval receive_timeout = 500000}; //0.5 seconds
  // setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));

  setsockopt(socket_handle_, SOL_CAN_RAW, CAN_RAW_FILTER, can_receive_filter.data(), can_receive_filter.size());

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): reinterpret cast required by syscall
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    printf("Error in socket bind");
    throw - 2;
  }
}

bool CanopenInterface::sdoDownload(RuntimeCommand command, uint8_t subindex, uint32_t data) {
  struct can_frame frame {};

  frame.can_id = sdo_cob_id_offset_ + roboteq::CanopenInterface::roboteq_can_id_;
  frame.can_dlc = CAN_FRAME_SIZE_BYTES_;
  // frame.data[0] = (sdo_command_ << 4) | (CanopenInterface::RUNTIME_COMMAND_MAP_.at(command).number_of_unused_bytes <<
  // 2);
  frame.data[0] = (sdo_command_ << 4) | (0 << 2);
  frame.data[1] = CanopenInterface::RUNTIME_COMMAND_MAP_.at(command).canopen_index;
  frame.data[2] = CanopenInterface::RUNTIME_COMMAND_MAP_.at(command).canopen_index >> bytesToBits(1);
  frame.data[3] = subindex;
  frame.data[4] = data;
  frame.data[5] = data >> bytesToBits(1);  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  frame.data[6] = data >> bytesToBits(2);  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  frame.data[7] = data >> bytesToBits(3);  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

  ssize_t bytes_written = write(roboteq::CanopenInterface::socket_handle_, &frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    // TODO: throw error
    return false;
  }

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  // std::cout <<  std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t" <<
  // static_cast<unsigned>(response_frame.data[0])
  //                         << "\t" << static_cast<unsigned>(response_frame.data[1]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[2]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[3])
  //                         << "\t" << static_cast<unsigned>(response_frame.data[4]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[5]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[6])
  //                         << "\t" << static_cast<unsigned>(response_frame.data[7]) << std::endl;

  if (bytes_read != sizeof(struct can_frame)) {
    // TODO: throw error
    return false;  // NOLINT(readability-simplify-boolean-expr): temp until error todo is finished
  }
  return true;
}

uint32_t CanopenInterface::sdoUpload(RuntimeQuery query, uint8_t subindex) {
  struct can_frame query_frame {};

  query_frame.can_id = sdo_cob_id_offset_ + roboteq::CanopenInterface::roboteq_can_id_;
  query_frame.can_dlc = CAN_FRAME_SIZE_BYTES_;
  query_frame.data[0] = (sdo_query_ << 4)
      /*|(CanopenInterface::RUNTINE_QUERY_MAP_.at(query).number_of_unused_bytes << 2)*/;  // TODO: fix
  query_frame.data[0] = (sdo_command_ << 4) | (0 << 2);
  query_frame.data[1] = CanopenInterface::RUNTINE_QUERY_MAP_.at(query).canopen_index;
  query_frame.data[2] = CanopenInterface::RUNTINE_QUERY_MAP_.at(query).canopen_index >> bytesToBits(1);
  query_frame.data[3] = subindex;
  query_frame.data[4] = 0;
  query_frame.data[5] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  query_frame.data[6] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  query_frame.data[7] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

  ssize_t bytes_written = write(roboteq::CanopenInterface::socket_handle_, &query_frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    // TODO: throw error
    return 0;
  }

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  // std::cout <<  std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t" <<
  // static_cast<unsigned>(response_frame.data[0])
  //                         << "\t" << static_cast<unsigned>(response_frame.data[1]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[2]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[3])
  //                         << "\t" << static_cast<unsigned>(response_frame.data[4]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[5]) << "\t" <<
  //                         static_cast<unsigned>(response_frame.data[6])
  //                         << "\t" << static_cast<unsigned>(response_frame.data[7]) << std::endl;

  if (bytes_read != sizeof(struct can_frame)) {
    // TODO: throw error
    return 0;
  }

  uint32_t response_data{};
  static constexpr size_t START_OF_DATA_INDEX{4};
  for (size_t data_index = START_OF_DATA_INDEX; data_index < CAN_FRAME_SIZE_BYTES_; data_index++) {
    response_data |= (response_frame.data[data_index] << (data_index - START_OF_DATA_INDEX));
  }

  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  return (response_data);
}

}  // namespace roboteq
