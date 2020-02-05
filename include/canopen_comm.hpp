#pragma once

#include <linux/can.h>

#include <unordered_map>

#include "i_comm.hpp"

namespace roboteq {

typedef struct {
  int canopen_index;
  int number_of_unused_bytes;
} command_properties_t;

class canopen_comm : public i_comm {
 public:
  explicit canopen_comm(canid_t roboteq_can_id = 0x1, const std::string& ifname = "can0");
  ~canopen_comm() {}

  bool sdo_download(send_runtime_command command, uint8_t subindex, uint32_t data) override;
  uint32_t sdo_upload(send_runtime_query query, uint8_t subindex) override;

 private:
  int roboteq_can_id_;
  int socket_handle_;
  const uint16_t sdo_command_ = 2;
  const uint16_t sdo_query_ = 4;
  const uint16_t sdo_cob_id_offset_ = 0x600;
  const uint16_t sdo_response_cob_id_offset_ = 0x580;
  static const std::unordered_map<send_runtime_command, command_properties_t> RUNTIME_COMMAND_MAP;
  static const std::unordered_map<send_runtime_query, command_properties_t> RUNTINE_QUERY_MAP;
};

}  // namespace roboteq
