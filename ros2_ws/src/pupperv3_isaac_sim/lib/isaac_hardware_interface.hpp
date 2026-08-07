#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "actuator_model.hpp"  // from pupper_mujoco_sim -- the SAME motor model
#include "fixed_size_queue.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace isaac_hardware_interface {

// Wire layout, shared with description/isaac_sim/isaac_bridge.py. Keep the two in step.
constexpr size_t kNumJoints = 12;
constexpr size_t kCmdLen = 5 * kNumJoints;              // pos_tgt, vel_tgt, ff, kp, kd
constexpr size_t kStateLen = 2 * kNumJoints + 10 + 6;   // pos, vel, imu(10), body_pos(3), body_vel(3)

/// ros2_control SystemInterface backed by an out-of-process Isaac Sim.
///
/// This mirrors MujocoHardwareInterface structurally, which is closer than it sounds:
/// MuJoCo's plugin calls mj_step() ZERO times. It spawns a free-running, wall-clock-paced
/// physics thread (mujoco_hardware_interface.cpp:143) and read()/write() only latch through
/// mutex-protected globals. So MuJoCo is ALREADY out-of-process from the plugin's point of
/// view, and this class occupies the same seam with DDS in place of the mutex:
///
///   MuJoCo:  write() -> latest_command_ --lock-->  physics thread
///   Isaac:   write() -> /isaac/joint_command --DDS--> isaac_bridge.py physics callback
///
/// read()/write() never block and never step, exactly as MuJoCo's do not.
class IsaacHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacHardwareInterface)

  virtual ~IsaacHardwareInterface();

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
  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Where the motor model runs. "isaac": send raw kp/kd/targets and let isaac_bridge.py run
  // the model on fresh per-step state (mirrors MuJoCo's update_ctrl_ before every mj_step).
  // "plugin": run it here and send tau as ff -- zero drift from the C++, but the PD then
  // closes at the controller rate on DDS-stale feedback. Must match the bridge's
  // --torque-mode.
  enum class TorqueSource { kIsaac, kPlugin };
  TorqueSource torque_source_ = TorqueSource::kIsaac;

  bool use_imu_ = true;

  // Own node + executor thread: a SystemInterface has no node of its own to spin.
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic<bool> spinning_{false};

  // Mirrors mujoco's `d` + mtx: newest snapshot, latched under a lock, never blocking.
  std::mutex state_mutex_;
  std::vector<double> latest_state_;
  std::atomic<bool> state_received_{false};

  std::unique_ptr<PIDActuatorModel> actuator_model_;  // only used when torque_source_=kPlugin

  // Latency queues. NOTE mujoco's *_latency_timesteps count CONTROL CYCLES, not physics
  // timesteps -- enqueue() runs once per read()/write(). Its 8/10 are therefore 40ms/50ms at
  // 200Hz, tuned against a ZERO-latency in-process sim. Over DDS the real transport latency
  // adds on top, so these default to 0 and must be re-tuned against a measurement.
  FixedSizeQueue<std::vector<double>> command_buffer_;
  FixedSizeQueue<std::vector<double>> imu_buffer_;

  // IMU state
  std::array<double, 4> hw_state_imu_orientation_;          // x, y, z, w
  std::array<double, 3> hw_state_imu_angular_velocity_;     // x, y, z
  std::array<double, 3> hw_state_imu_linear_acceleration_;  // x, y, z
  double imu_roll_, imu_pitch_, imu_yaw_;

  // Ground truth, mirroring mujoco_sensor/body_pos|body_vel. Nothing consumes it; it is what
  // the MuJoCo-vs-Isaac comparison diffs.
  std::array<double, 3> hw_state_isaac_body_pos_;
  std::array<double, 3> hw_state_isaac_body_vel_;

  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;

  std::vector<double> hw_command_positions_;
  std::vector<double> hw_command_velocities_;
  std::vector<double> hw_command_efforts_;
  std::vector<double> hw_command_kps_;
  std::vector<double> hw_command_kds_;
};

}  // namespace isaac_hardware_interface
