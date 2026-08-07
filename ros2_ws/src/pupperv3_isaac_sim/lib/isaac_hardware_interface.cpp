#include "isaac_hardware_interface.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace isaac_hardware_interface {

namespace {
constexpr const char *kLogger = "IsaacHardwareInterface";

double get_param(const hardware_interface::HardwareInfo &info, const std::string &name) {
  return std::stod(info.hardware_parameters.at(name));
}

double get_param_or(const hardware_interface::HardwareInfo &info, const std::string &name,
                    double fallback) {
  return info.hardware_parameters.count(name) ? std::stod(info.hardware_parameters.at(name))
                                              : fallback;
}
}  // namespace

IsaacHardwareInterface::~IsaacHardwareInterface() {
  on_deactivate(rclcpp_lifecycle::State());
  spinning_ = false;
  if (executor_) executor_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

hardware_interface::CallbackReturn IsaacHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "expected %zu joints, got %zu", kNumJoints,
                 info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_state_positions_.resize(info_.joints.size(), 0.0);
  hw_state_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_positions_.resize(info_.joints.size(), 0.0);
  hw_command_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_efforts_.resize(info_.joints.size(), 0.0);
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  hw_command_kds_.resize(info_.joints.size(), 0.0);

  // Per-joint <param>s. MuJoCo reads all of these with .at() and then uses none of them;
  // they are read here for the same reason -- so a components.xacro that works with the
  // MuJoCo plugin also works with this one, and a missing param fails loudly at load.
  for (const auto &joint : info_.joints) {
    for (const char *p : {"position_min", "position_max", "velocity_max", "effort_max", "kp_max",
                          "kd_max"}) {
      if (!joint.parameters.count(p)) {
        RCLCPP_FATAL(rclcpp::get_logger(kLogger), "joint %s missing <param name=\"%s\">",
                     joint.name.c_str(), p);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  const std::string src = info_.hardware_parameters.count("torque_source")
                              ? info_.hardware_parameters.at("torque_source")
                              : "isaac";
  if (src == "isaac") {
    torque_source_ = TorqueSource::kIsaac;
  } else if (src == "plugin") {
    torque_source_ = TorqueSource::kPlugin;
  } else {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger),
                 "torque_source must be 'isaac' or 'plugin', got '%s'", src.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Same values MuJoCo feeds its PIDActuatorModel (pupper_v3_mujoco.ros2_control.xacro:10-14).
  // Required, not defaulted: a silently-defaulted motor model is a silently different robot,
  // which is exactly what this whole comparison is trying to detect.
  ActuatorParams params(0.0,  // kp -- overwritten per call by run()
                        0.0,  // kd
                        get_param(info_, "bus_voltage"), get_param(info_, "kt"),
                        get_param(info_, "phase_resistance"), get_param(info_, "saturation_torque"),
                        get_param(info_, "software_torque_limit"));
  actuator_model_ = std::make_unique<PIDActuatorModel>(params);

  if (info_.hardware_parameters.count("use_imu")) {
    const std::string v = info_.hardware_parameters.at("use_imu");
    use_imu_ = !(v == "false" || v == "False");
  }

  // Parsed and stored to mirror MuJoCo, which also never uses them: the mount correction is
  // real-robot-only (control_board_hardware_interface.cpp:316-325). Isaac's IMU is already
  // base_link-frame, so applying it here would double-correct by 135 degrees.
  if (!info_.sensors.empty()) {
    imu_roll_ = std::stod(info_.sensors[0].parameters.at("roll"));
    imu_pitch_ = std::stod(info_.sensors[0].parameters.at("pitch"));
    imu_yaw_ = std::stod(info_.sensors[0].parameters.at("yaw"));
  }

  const int cmd_lat = static_cast<int>(get_param_or(info_, "command_latency_timesteps", 0));
  const int imu_lat = static_cast<int>(get_param_or(info_, "imu_latency_timesteps", 0));
  command_buffer_.fill(cmd_lat, std::vector<double>(kCmdLen, 0.0));
  imu_buffer_.fill(imu_lat, std::vector<double>(10, 0.0));
  if (cmd_lat > 0 || imu_lat > 0) {
    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "latency queues cmd=%d imu=%d control cycles, ADDING to real DDS transport. "
                "Transport was measured sub-cycle (~0.12 cycle one-way), so 8/10 matches "
                "MuJoCo's total; see pupper_v3_isaac.ros2_control.xacro.",
                cmd_lat, imu_lat);
  }

  latest_state_.assign(kStateLen, 0.0);

  // A SystemInterface has no node to spin, so own one.
  if (!rclcpp::ok()) {
    RCLCPP_FATAL(rclcpp::get_logger(kLogger), "rclcpp not initialised");
    return hardware_interface::CallbackReturn::ERROR;
  }
  node_ = std::make_shared<rclcpp::Node>("isaac_hardware_interface");
  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/isaac/joint_command", 10);
  state_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/isaac/joint_state", 10,
      std::bind(&IsaacHardwareInterface::state_callback, this, std::placeholders::_1));
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spinning_ = true;
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "initialised, torque_source=%s", src.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void IsaacHardwareInterface::state_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != kStateLen) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLogger), *node_->get_clock(), 1000,
                          "joint_state len %zu != %zu; ignoring", msg->data.size(), kStateLen);
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_.assign(msg->data.begin(), msg->data.end());
  state_received_ = true;
}

hardware_interface::CallbackReturn IsaacHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (size_t i = 0; i < hw_state_positions_.size(); i++) {
    hw_state_positions_[i] = 0.0;
    hw_state_velocities_[i] = 0.0;
    hw_command_positions_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    // Match MuJoCo (mujoco_hardware_interface.cpp:210-211), which seeds live gains rather
    // than the real robot's 0.0.
    hw_command_kps_[i] = 7.5;
    hw_command_kds_[i] = 0.5;
  }
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IsaacHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
  }
  // Joint effort state is deliberately NOT exported -- MuJoCo comments it out
  // (mujoco_hardware_interface.cpp:160-162). Only the real robot exports it.

  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x",
                                                                   &hw_state_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y",
                                                                   &hw_state_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z",
                                                                   &hw_state_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w",
                                                                   &hw_state_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

  // Mirrors mujoco_sensor/* (mujoco_hardware_interface.cpp:186-197). Not declared in any
  // xacro; exported programmatically, which the ResourceManager accepts.
  state_interfaces.emplace_back(hardware_interface::StateInterface("isaac_sensor", "body_pos.x",
                                                                   &hw_state_isaac_body_pos_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("isaac_sensor", "body_pos.y",
                                                                   &hw_state_isaac_body_pos_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("isaac_sensor", "body_pos.z",
                                                                   &hw_state_isaac_body_pos_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("isaac_sensor", "body_vel.x",
                                                                   &hw_state_isaac_body_vel_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("isaac_sensor", "body_vel.y",
                                                                   &hw_state_isaac_body_vel_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("isaac_sensor", "body_vel.z",
                                                                   &hw_state_isaac_body_vel_[2]));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IsaacHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_command_kps_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_command_kds_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn IsaacHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!state_received_) {
    RCLCPP_WARN(rclcpp::get_logger(kLogger),
                "activating before any /isaac/joint_state arrived -- is isaac_bridge.py running, "
                "and on the same RMW? (bridge defaults to cyclone via run_isaac_bridge.sh)");
  }
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Actually zero the gains. MuJoCo's equivalent is `hw_command_kps_.resize(n, 0.0)`, which
  // is a no-op: resize() to the current size leaves existing elements untouched. Not copied.
  std::fill(hw_command_kps_.begin(), hw_command_kps_.end(), 0.0);
  std::fill(hw_command_kds_.begin(), hw_command_kds_.end(), 0.0);
  std::fill(hw_command_efforts_.begin(), hw_command_efforts_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IsaacHardwareInterface::read(const rclcpp::Time & /*time*/,
                                                             const rclcpp::Duration & /*period*/) {
  std::vector<double> s;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    s = latest_state_;  // newest snapshot; never block, never step -- same as MuJoCo
  }

  for (size_t i = 0; i < kNumJoints; i++) {
    hw_state_positions_[i] = s[i];
    hw_state_velocities_[i] = s[kNumJoints + i];
  }

  // One enqueue per read(), matching MuJoCo (mujoco_hardware_interface.cpp:274) -- which is
  // why its "timesteps" are really control cycles.
  std::vector<double> imu(s.begin() + 2 * kNumJoints, s.begin() + 2 * kNumJoints + 10);
  const std::vector<double> lagged_imu = imu_buffer_.enqueue(imu);

  if (use_imu_) {
    for (size_t i = 0; i < 4; i++) hw_state_imu_orientation_[i] = lagged_imu[i];      // x,y,z,w
    for (size_t i = 0; i < 3; i++) hw_state_imu_angular_velocity_[i] = lagged_imu[4 + i];
    for (size_t i = 0; i < 3; i++) hw_state_imu_linear_acceleration_[i] = lagged_imu[7 + i];
  } else {
    hw_state_imu_orientation_ = {0.0, 0.0, 0.0, 1.0};
    hw_state_imu_angular_velocity_ = {0.0, 0.0, 0.0};
    hw_state_imu_linear_acceleration_ = {0.0, 0.0, 0.0};
  }

  for (size_t i = 0; i < 3; i++) {
    hw_state_isaac_body_pos_[i] = s[2 * kNumJoints + 10 + i];
    hw_state_isaac_body_vel_[i] = s[2 * kNumJoints + 13 + i];
  }

  // No NaN check and no ERROR return: matches MuJoCo, which has neither. The real robot does
  // check (control_board_hardware_interface.cpp:343-356).
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacHardwareInterface::write(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {
  std::vector<double> cmd(kCmdLen, 0.0);

  // No clamping to position_min/max, velocity_max, effort_max, kp_max, kd_max. MuJoCo clamps
  // nothing (its write() is bare); only the real robot clamps. Matching MuJoCo is the point.
  for (size_t i = 0; i < kNumJoints; i++) {
    if (torque_source_ == TorqueSource::kPlugin) {
      // Run the model here, on the last state read() latched -- i.e. DDS-stale feedback, and
      // at the controller rate rather than the physics rate. Send the torque as ff and zero
      // the gains so the bridge (in --torque-mode passthrough) applies it verbatim.
      const double tau = actuator_model_->run(hw_command_kps_[i], hw_command_kds_[i],
                                              hw_state_positions_[i], hw_command_positions_[i],
                                              hw_state_velocities_[i], hw_command_velocities_[i],
                                              hw_command_efforts_[i]);
      cmd[0 * kNumJoints + i] = hw_command_positions_[i];
      cmd[1 * kNumJoints + i] = hw_command_velocities_[i];
      cmd[2 * kNumJoints + i] = tau;
      cmd[3 * kNumJoints + i] = 0.0;
      cmd[4 * kNumJoints + i] = 0.0;
    } else {
      cmd[0 * kNumJoints + i] = hw_command_positions_[i];
      cmd[1 * kNumJoints + i] = hw_command_velocities_[i];
      cmd[2 * kNumJoints + i] = hw_command_efforts_[i];
      cmd[3 * kNumJoints + i] = hw_command_kps_[i];
      cmd[4 * kNumJoints + i] = hw_command_kds_[i];
    }
  }

  const std::vector<double> lagged = command_buffer_.enqueue(cmd);

  std_msgs::msg::Float64MultiArray msg;
  msg.data = lagged;
  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace isaac_hardware_interface

PLUGINLIB_EXPORT_CLASS(isaac_hardware_interface::IsaacHardwareInterface,
                       hardware_interface::SystemInterface)
