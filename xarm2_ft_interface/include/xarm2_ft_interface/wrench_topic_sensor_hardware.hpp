// Copyright (c) 2026 Hivebotics
// BSD License

#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace xarm2_ft_interface
{

/**
 * @brief A ros2_control SensorInterface that subscribes to a WrenchStamped topic
 *        and exposes force/torque values as hardware state interfaces.
 *
 * Real hardware uses the xArm API WrenchStamped stream. Simulation uses the
 * native Gazebo FT interfaces instead of loading this adapter.
 *
 * URDF ros2_control parameters:
 *   - wrench_topic (string): The topic to subscribe to. Default: "/ft_sensor/wrench"
 *   - expected_frame (string): Required incoming message frame ID.
 *   - sensor_timeout (double): Maximum receipt age in seconds, default 0.5.
 *
 * The sensor must be declared in the URDF with name "force_torque_sensor" and
 * the 6 standard state interfaces: force.x, force.y, force.z, torque.x, torque.y, torque.z
 *
 * Uses SystemInterface for the existing real-hardware resource manager setup.
 * No command interfaces are exported. Missing or invalid data fails closed.
 */
class WrenchTopicSensorHardware : public hardware_interface::SystemInterface
{
public:
  WrenchTopicSensorHardware() = default;
  ~WrenchTopicSensorHardware() override;

  // Lifecycle
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read from the subscribed topic
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // No-op write (read-only sensor)
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  friend class WrenchTopicSensorHardwareTestPeer;

  struct Sample
  {
    std::array<double, 6> wrench{};
    std::chrono::steady_clock::time_point received_at{};
    bool valid{false};
  };

  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void stop_executor();
  bool sample_is_fresh(const Sample & sample) const;

  // Internal ROS node for subscribing
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::mutex activation_mutex_;
  std::condition_variable sample_received_;

  // Topic name (read from URDF param, overridable as ROS param)
  std::string wrench_topic_;
  std::string expected_frame_;
  std::chrono::duration<double> sensor_timeout_{0.5};

  // Sensor name from URDF
  std::string sensor_name_;

  // State interface storage: [force.x, force.y, force.z, torque.x, torque.y, torque.z]
  std::array<double, 6> hw_sensor_states_{};

  // Latest received values (written by callback, read by read())
  realtime_tools::RealtimeBuffer<Sample> sample_buffer_;
};

}  // namespace xarm2_ft_interface
