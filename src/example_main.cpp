#include <memory>
#include <iostream>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"
#include "rawcan_comm.hpp"

int main(int argc, char **argv) {

    std::cout << "Start! " << std::endl;

    std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::rawcan_comm>(0x600, "can0");  
    roboteq::roboteq_controller motor_controller(std::move(comm));

    motor_controller.SetVelocity(0x10, 0x01-01);
    motor_controller.SetAcceleration(0x10, 0x01-01);
    // motor_controller.ReadBatteryAmps(0x01);

    std::cout << "End! " << std::endl;

    return 0;
}