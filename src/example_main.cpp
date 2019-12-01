#include <memory>

#include "i_comm.hpp"

int main(int argc, char **argv) {

    std::unique_ptr<i_comm> comm = std::make_unique<rawcan_comm>(0x100, "can0");  
    roboteq_controller::roboteq_controller motor_controller = roboteq_controller(std::move(comm)); 

    return 0
}