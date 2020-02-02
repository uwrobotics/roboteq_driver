#include <iostream>
#include <memory>

#include "canopen_comm.hpp"
#include "i_comm.hpp"
#include "roboteq_controller.hpp"

int main(void) {
  std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::canopen_comm>(0x01, "can0");
  roboteq::roboteq_controller motor_controller(std::move(comm));

  for (int i = 0; i < 10000; i++) {
    bool release_shutdown = motor_controller.ReleaseShutdown();
    std::cout << release_shutdown << std::endl;

    bool set_motor_command = motor_controller.SetMotorCommand(200, 1);
    std::cout << "set_motor_command successful:" << set_motor_command << std::endl;

    bool set_velocity = motor_controller.SetVelocity(200, 1);
    std::cout << "set_velocity successful:" << set_velocity << std::endl;
    int32_t encoder_motor_speed = motor_controller.ReadEncoderMotorSpeed(1);
    std::cout << "encoder_motor_speed: " << encoder_motor_speed << std::endl;

    int16_t feedback = motor_controller.ReadFeedback(1);
    std::cout << "feedback successful:" << feedback << std::endl;
  }

  // // set methods

  // bool set_position = motor_controller.SetPosition(300,1);
  // std::cout << set_position << std::endl;

  // bool set_velocity = motor_controller.SetVelocity(100, 2);
  // std::cout << set_velocity << std::endl;

  // bool set_encoder_counter = motor_controller.SetEncoderCounter(100, 2);
  // std::cout << set_encoder_counter << std::endl;

  // bool set_brushless_counter = motor_controller.SetBrushlessCounter(100, 2);
  // std::cout << set_brushless_counter << std::endl;

  // // // bool set_user_int_variable = motor_controller.SetUserIntVariable(int32_t var, uint8_t nbvar);
  // // // std::cout << set_user_int_variable << std::endl;

  // bool set_acceleration = motor_controller.SetAcceleration(100, 2);
  // std::cout << set_acceleration << std::endl;

  // bool set_deceleration = motor_controller.SetDeceleration(100, 2);
  // std::cout << set_deceleration << std::endl;

  // bool set_all_digital_out_bits = motor_controller.SetAllDigitalOutBits(1);
  // std::cout << set_all_digital_out_bits << std::endl;

  // // // bool set_individual_digital_out_bits = motor_controller.SetIndividualDigitalOutBits(uint8_t out_bits);
  // // // std::cout << set_individual_digital_out_bits << std::endl;

  // // // bool reset_individual_out_bits = motor_controller.ResetIndividualOutBits(uint8_t out_bits);
  // // // std::cout << reset_individual_out_bits << std::endl;

  // bool load_home_counter = motor_controller.LoadHomeCounter(2);
  // std::cout << load_home_counter << std::endl;

  // bool emergency_shutdown = motor_controller.EmergencyShutdown();
  // std::cout << emergency_shutdown << std::endl;

  // bool release_shutdown = motor_controller.ReleaseShutdown();
  // std::cout << release_shutdown << std::endl;

  // bool stop_in_all_modes = motor_controller.StopInAllModes(2);
  // std::cout << stop_in_all_modes << std::endl;

  // bool set_pos_relative = motor_controller.SetPosRelative(100, 2);
  // std::cout << set_pos_relative << std::endl;

  // bool set_next_pos_absolute = motor_controller.SetNextPosAbsolute(100, 2);
  // std::cout << set_next_pos_absolute << std::endl;

  // bool set_next_pos_relative = motor_controller.SetNextPosRelative(100, 2);
  // std::cout << set_next_pos_relative << std::endl;

  // bool set_next_acceleration = motor_controller.SetNextAcceleration(100, 2);
  // std::cout << set_next_acceleration << std::endl;

  // bool set_next_deceleration = motor_controller.SetNextDeceleration(100, 2);
  // std::cout << set_next_deceleration << std::endl;

  // bool set_next_velocity = motor_controller.SetNextVelocity(100, 2);
  // std::cout << set_next_velocity << std::endl;

  // // //  bool set_user_bool_variable = motor_controller.SetUserBoolVariable(uint32_t var, uint8_t nbvar);
  // // //  std::cout << set_user_bool_variable << std::endl;

  // bool save_config_to_flash = motor_controller.SaveConfigToFlash();
  // std::cout << save_config_to_flash << std::endl;

  // read methods
  //     int16_t motor_amps = motor_controller.ReadMotorAmps(2);
  //     std::cout << motor_amps << std::endl;

  // int16_t motor_command = motor_controller.ReadActualMotorCommand(2);
  // std::cout << "motor_command:" << motor_command << std::endl;

  //     int16_t applied_power_level = motor_controller.ReadAppliedPowerLevel(2);
  //     std::cout << applied_power_level << std::endl;

  //     int32_t encoder_motor_speed = motor_controller.ReadEncoderMotorSpeed(2);
  //     std::cout << encoder_motor_speed << std::endl;

  // int32_t absolute_encoder_count = motor_controller.ReadAbsoluteEncoderCount(2);
  // std::cout << absolute_encoder_count << std::endl;

  //     int32_t brushless_counter = motor_controller.ReadAbsoluteBrushlessCounter(2);
  //     std::cout << brushless_counter << std::endl;

  //     // // int32_t user_interger_varaible = motor_controller.ReadUserIntegerVariable(int32_t nbvar);
  //     // // std::cout << user_interger_varaible << std::endl;

  //     int16_t motor_speed_relative_to_max = motor_controller.ReadEncoderMotorSpeedRelativeToMaxSpeed(2);
  //     std::cout << motor_speed_relative_to_max << std::endl;

  //     int32_t encoder_count_relative = motor_controller.ReadEncoderCountRelative(2);
  //     std::cout << encoder_count_relative << std::endl;

  //     int32_t brushless_count_relative = motor_controller.ReadBrushlessCountRelative(2);
  //     std::cout << brushless_count_relative << std::endl;

  //     int16_t brushless_motor_speed = motor_controller.ReadBrushlessMotorSpeed(2);
  //     std::cout << brushless_motor_speed << std::endl;

  //     int16_t brushless_motor_speed_relative_to_max_speed =
  //     motor_controller.ReadBrushlessMotorSpeedRelativeToMaxSpeed(2); std::cout <<
  //     brushless_motor_speed_relative_to_max_speed << std::endl;

  //     int16_t battery_amps = motor_controller.ReadBatteryAmps(2);
  //     std::cout << battery_amps << std::endl;

  //     uint16_t internal_voltages = motor_controller.ReadInternalVoltages(1); // 1 = Internal volts, 2 = Battery
  //     volts, 3 = 5V output std::cout << internal_voltages << std::endl;

  //     uint32_t all_digital_inputs = motor_controller.ReadAllDigitalInputs();
  //     std::cout << all_digital_inputs << std::endl;

  // int8_t case_and_internal_temperature = motor_controller.ReadCaseAndInternalTemperatures(2); // Total Number of
  // Motors + 1 std::cout << case_and_internal_temperature << std::endl;

  //     int16_t feedback = motor_controller.ReadFeedback(2);
  //     std::cout << feedback << std::endl;

  //     uint16_t status_flags = motor_controller.ReadStatusFlags();
  //     std::cout << status_flags << std::endl;

  //     uint16_t fault_flags = motor_controller.ReadFaultFlags();
  //     std::cout << fault_flags << std::endl;

  //     uint16_t current_digital_outputs = motor_controller.ReadCurrentDigitalOutputs();
  //     std::cout << current_digital_outputs << std::endl;

  //     int32_t closed_loop_error = motor_controller.ReadClosedLoopError(2);
  //     std::cout << closed_loop_error << std::endl;

  //     // // bool user_boolean_variable = motor_controller.ReadUserBooleanVariable(uint32_t nbvar);
  //    // // std::cout << user_boolean_variable << std::endl;

  //     int32_t internal_serial_command = motor_controller.ReadInternalSerialCommand(2);
  //     std::cout << internal_serial_command << std::endl;

  //     int32_t internal_analog_command = motor_controller.ReadInternalAnalogCommand(2);
  //     std::cout << internal_analog_command << std::endl;

  //     int32_t internal_pulse_command = motor_controller.ReadInternalInternalPulseCommand(2);
  //     std::cout << internal_pulse_command << std::endl;

  //     uint32_t time = motor_controller.ReadTime(1); // 1 : Seconds, 2 : Minutes, 3 : Hours (24h format), 4 :
  //     Dayofmonth, 5 : Month, 6 : Year in full, std::cout << time << std::endl;

  //     uint16_t spektrum_radio_capture = motor_controller.ReadSpektrumRadioCapture(1);
  //     std::cout << spektrum_radio_capture << std::endl;

  //     uint8_t destination_position_reached_flag = motor_controller.ReadDestinationPositionReachedFlag(2);
  //     std::cout << destination_position_reached_flag << std::endl;

  //     int32_t mems_accelerometer_axis = motor_controller.ReadMEMSAccelerometerAxis(4); // 2 * Total Number of Motors
  //     std::cout << mems_accelerometer_axis << std::endl;

  //     uint8_t mag_sensor_track_detect = motor_controller.ReadMagsensorTrackDetect();
  //     std::cout << mag_sensor_track_detect << std::endl;

  //     // // int16_t mag_sensor_track_position = motor_controller.ReadMagsensorTrackPosition(uint8_t nb_pulse); //  3
  //     * Total Number of Pulse Inputs
  //     // //  std::cout << mag_sensor_track_position << std::endl;

  //     // // uint8_t mag_sensor_markers = motor_controller.ReadMagsensorMarkers(uint8_t nb_pulse);  // 2 * Total
  //     Number of Pulse Inputs
  //     // //  std::cout << mag_sensor_markers << std::endl;

  //     // // uint16_t mag_sensor_status = motor_controller.ReadMagsensorStatus(uint8_t nb_pulse);  //  Total Number of
  //     Pulse Inputs
  //     // //  std::cout << mag_sensor_status << std::endl;

  //     // // uint16_t motor_status_flags = motor_controller.ReadMotorStatusFlags(uint8_t nb_pulse); //  Total Number
  //     of Pulse Inputs
  //     // //  std::cout << motor_status_flags << std::endl;

  return 0;
}
