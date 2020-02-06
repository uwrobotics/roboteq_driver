#pragma once

#include <linux/can.h>

#include <unordered_map>

#include "CommunicationInterface.hpp"

namespace roboteq {

using command_properties_t = struct {
  int canopen_index;
  int number_of_unused_bytes;
};

class CanopenInterface : public CommunicationInterface {
 public:
  explicit CanopenInterface(canid_t roboteq_can_id = 0x1, const std::string& ifname = "can0");

  bool sdoDownload(RuntimeCommand command, uint8_t subindex, uint32_t data) override;
  uint32_t sdoUpload(RuntimeQuery query, uint8_t subindex) override;

 private:
  int roboteq_can_id_;
  int socket_handle_;

  static const std::unordered_map<RuntimeCommand, command_properties_t> RUNTIME_COMMAND_MAP_;
  static const std::unordered_map<RuntimeQuery, command_properties_t> RUNTINE_QUERY_MAP_;

  static constexpr uint16_t sdo_command_{2};
  static constexpr uint16_t sdo_query_{4};
  static constexpr uint16_t sdo_cob_id_offset_{0x600};
  static constexpr uint16_t sdo_response_cob_id_offset_{0x580};
  static constexpr uint8_t CAN_FRAME_SIZE_BYTES_{8};

  static inline constexpr unsigned bytesToBits(const unsigned num_bytes) {
    constexpr unsigned bits_per_byte = 8;
    return num_bytes * bits_per_byte;
  }
};

}  // namespace roboteq
