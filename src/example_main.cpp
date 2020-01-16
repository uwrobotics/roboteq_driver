#include <memory>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"
#include "rawcan_comm.hpp"

int main(int argc, char **argv) {

    std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::rawcan_comm>(0x600, "vcan0");  
    roboteq::roboteq_controller motor_controller(std::move(comm));

    motor_controller.SetNextVelocity(0x01, 0x01-02);
    motor_controller.ReadMotorAmps(0x01);

    return 0;
}