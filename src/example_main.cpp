#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "canopen_comm.hpp"
#include "i_comm.hpp"
#include "roboteq_controller.hpp"

static constexpr int32_t TEST_SPEED{500};

int main() {
  std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::canopen_comm>(0x01, "can0");
  roboteq::roboteq_controller motor_controller(std::move(comm));

  static constexpr int NUM_DRIVE_JOINTS{2};
  std::array<double, NUM_DRIVE_JOINTS> positions{};
  std::array<double, NUM_DRIVE_JOINTS> velocities{};
  std::array<double, NUM_DRIVE_JOINTS> efforts{};

  using namespace std::chrono_literals;
  static constexpr auto EXECUTION_PERIOD{1s / 100};  // 100 Hz
  auto start_time = std::chrono::steady_clock::now();
  auto end_time = start_time + 15s;
  auto next_execution_time = start_time;
  while (std::chrono::steady_clock::now() < end_time) {
    // read to all joints
    for (int joint_num = 0; joint_num < NUM_DRIVE_JOINTS; joint_num++) {
      positions[joint_num] = motor_controller.readAbsoluteEncoderCount(joint_num);
      velocities[joint_num] = motor_controller.readEncoderMotorSpeed(joint_num);
      efforts[joint_num] = motor_controller.readMotorAmps(joint_num);

      static constexpr int NUMBER_FIELD_WIDTH{5};
      std::cout << std::left << "joint_num: " << std::setw(NUMBER_FIELD_WIDTH) << joint_num
                << "position: " << std::setw(NUMBER_FIELD_WIDTH) << positions[joint_num]
                << "velocity: " << std::setw(NUMBER_FIELD_WIDTH) << velocities[joint_num]
                << "effort: " << std::setw(NUMBER_FIELD_WIDTH) << efforts[joint_num] << std::endl;
    }

    // write to all joints
    for (int joint_num = 0; joint_num < NUM_DRIVE_JOINTS; joint_num++) {
      bool command_successful = motor_controller.setVelocity(TEST_SPEED, joint_num);
      if (!command_successful) {
        bool shutdown_successful{false};
        do {
          shutdown_successful = motor_controller.emergencyShutdown();
          std::cout << "Command write failed. Sending ESTOP!" << std::endl;
        } while (!shutdown_successful);
        std::cout << "Test FAILED. Aborting program early." << std::endl;
        return EXIT_FAILURE;
      }
    }

    next_execution_time += EXECUTION_PERIOD;
    std::this_thread::sleep_until(next_execution_time);
  }

  std::cout << "Test SUCCESS! Program Terminating." << std::endl;
  return EXIT_SUCCESS;
}
