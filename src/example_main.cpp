#include <memory>
#include <iostream>

#include "roboteq_controller.hpp"
#include "i_comm.hpp"
#include "canopen_comm.hpp"

int main(int argc, char **argv) {

    std::cout << "Start! " << std::endl;

    std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::canopen_comm>(0x01, "can0");  
    roboteq::roboteq_controller motor_controller(std::move(comm));

    // motor_controller.SetVelocity(100, 2);
    // motor_controller.SetAcceleration(0x10, 0x01-01);
    // int16_t battery_amps = motor_controller.ReadBatteryAmps(1);
    uint16_t internal_voltages = motor_controller.ReadInternalVoltages(2);

    std::cout << internal_voltages << std::endl;

    return 0;
}