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
  explicit canopen_comm(canid_t roboteq_can_id = 0x1, std::string ifname = "can0");
  ~canopen_comm() {}

  bool sdo_download(send_runtime_command command, uint8_t subindex, uint32_t data) override;
  uint32_t sdo_upload(send_runtime_query query, uint8_t subindex) override;

 private:
  int _roboteq_can_id;
  int _socket_handle;
  const uint16_t _sdo_command = 2;
  const uint16_t _sdo_query = 4;
  const uint16_t _sdo_cob_id_offset = 0x600;
  const uint16_t _sdo_response_cob_id_offset = 0x580;
  static const std::unordered_map<send_runtime_command, command_properties_t> _runtime_command_map;
  static const std::unordered_map<send_runtime_query, command_properties_t> _runtime_query_map;
};

}  // namespace roboteq
