#include <memory>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"
#include "rawcan_comm.hpp"

int main(int argc, char **argv) {

    std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::rawcan_comm>(0x100, "can0");  
    roboteq::roboteq_controller motor_controller(std::move(comm));
    return 0;
}