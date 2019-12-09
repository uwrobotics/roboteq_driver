#include <memory>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"
#include "rawcan_comm.hpp"

int main(int argc, char **argv) {

    std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::rawcan_comm>(0x100, "vcan0");  
    roboteq::roboteq_controller motor_controller(std::move(comm));

    motor_controller.download(roboteq::runtime_command::SET_MOTOR_COMMAND, 0x02, 0x0A);

    return 0;
}