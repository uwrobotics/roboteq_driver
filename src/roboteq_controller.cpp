#include <memory>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"

namespace roboteq {

     roboteq_controller::roboteq_controller(std::unique_ptr<i_comm> &&comm) : _comm(std::move(comm)) {
    }

} // namespace roboteq
