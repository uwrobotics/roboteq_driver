#include <memory>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"

namespace roboteq {

     roboteq_controller::roboteq_controller(std::unique_ptr<i_comm> &&comm) : _comm(std::move(comm)) {
    }

    void Roboteq::SetPosition(long _position, uint8_t _channel)
        {
            i_comm::download(runtime_command command, uint8_t subindex, uint32_t data);
        }

} // namespace roboteq
