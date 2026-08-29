#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fixed_size_queue.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mujoco_core_interactive.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define IMU_I2C_DEVICE_NUMBER 1

namespace mujoco_hardware_interface {
class MujocoHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoHardwareInterface)

  virtual ~MujocoHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  bool use_imu_ = true;

  // IMU state
  FixedSizeQueue<std::vector<double>> imu_buffer_;
  std::array<double, 4> hw_state_imu_orientation_;          // x, y, z, w
  std::array<double, 3> hw_state_imu_angular_velocity_;     // x, y, z
  std::array<double, 3> hw_state_imu_linear_acceleration_;  // x, y, z
  double imu_roll_, imu_pitch_, imu_yaw_;

  std::array<double, 3> hw_state_mujoco_body_pos_;  // x, y, z
  std::array<double, 3> hw_state_mujoco_body_vel_;  // x, y, z

  // Actuator homing
  std::vector<int> hw_actuator_homing_stages_;
  std::vector<double> hw_actuator_homing_velocities_;
  std::vector<double> hw_actuator_homing_kps_;
  std::vector<double> hw_actuator_homing_kds_;
  std::vector<double> hw_actuator_homed_positions_;
  std::vector<double> hw_actuator_zero_positions_;
  std::vector<double> hw_actuator_homing_torque_thresholds_;
  std::vector<bool> hw_actuator_is_homed_;

  // Actuator limits
  std::vector<double> hw_actuator_position_mins_;
  std::vector<double> hw_actuator_position_maxs_;
  std::vector<double> hw_actuator_velocity_maxs_;
  std::vector<double> hw_actuator_effort_maxs_;
  std::vector<double> hw_actuator_kp_maxs_;
  std::vector<double> hw_actuator_kd_maxs_;

  // Actuator states
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;

  // Actuator commands
  FixedSizeQueue<mujoco_interactive::ActuatorCommand> command_buffer_;
  std::vector<double> hw_command_positions_;
  std::vector<double> hw_command_velocities_;
  std::vector<double> hw_command_efforts_;
  std::vector<double> hw_command_kps_;
  std::vector<double> hw_command_kds_;

  // T-Monitor: applied-torque publisher. Only wired up when capture is enabled (T_MONITOR
  // env set by launch). A SystemInterface has no node of its own, so -- like the Isaac
  // interface -- we own one and spin it on a thread. Publishes /applied_torque, one batched
  // message per read() carrying every physics-step sample drained from the sim core.
  bool torque_monitor_ = false;
  rclcpp::Node::SharedPtr t_monitor_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr t_monitor_pub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr t_monitor_executor_;
  std::thread t_monitor_spin_thread_;
  std::atomic<bool> t_monitor_spinning_{false};
};

}  // namespace mujoco_hardware_interface
