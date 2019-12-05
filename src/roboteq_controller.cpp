#include <memory>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"

namespace roboteq {

     roboteq_controller::roboteq_controller(std::unique_ptr<i_comm> &&comm) : _comm(std::move(comm)) {

    }

    // // set methods
    // void roboteq_controller::SetParameters(int16_t velocity, long accel, long decel, uint8_t channel) {

    //     _comm->SetParameters(velocity, accel, decel, channel);
    // }

} // namespace roboteq
