#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include "xarm2_ft_interface/wrench_topic_sensor_hardware.hpp"

namespace xarm2_ft_interface
{

class WrenchTopicSensorHardwareTestPeer
{
public:
  static void receive(WrenchTopicSensorHardware & hardware, double force,
    const std::string & frame = "uf_ft_sensor_ext_data")
  {
    hardware.expected_frame_ = "uf_ft_sensor_ext_data";
    auto message = std::make_shared<geometry_msgs::msg::WrenchStamped>();
    message->header.frame_id = frame;
    message->wrench.force.x = force;
    message->wrench.force.y = -2.0;
    message->wrench.force.z = 3.0;
    message->wrench.torque.x = -4.0;
    message->wrench.torque.y = 5.0;
    message->wrench.torque.z = -6.0;
    hardware.wrench_callback(message);
  }

  static void expire(WrenchTopicSensorHardware & hardware)
  {
    auto sample = *hardware.sample_buffer_.readFromRT();
    sample.received_at -= std::chrono::seconds(1);
    hardware.sample_buffer_.writeFromNonRT(sample);
  }
};

TEST(WrenchTopicSensorHardware, MissingDataReturnsError)
{
  WrenchTopicSensorHardware hardware;
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  for (const auto & interface : hardware.export_state_interfaces()) {
    EXPECT_TRUE(std::isnan(interface.get_value()));
  }
}

TEST(WrenchTopicSensorHardware, PreservesSixAxisSignsAndOrdering)
{
  WrenchTopicSensorHardware hardware;
  WrenchTopicSensorHardwareTestPeer::receive(hardware, 1.0);
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  const std::array<double, 6> expected{1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
  const auto interfaces = hardware.export_state_interfaces();
  ASSERT_EQ(interfaces.size(), expected.size());
  for (size_t index = 0; index < expected.size(); ++index) {
    EXPECT_DOUBLE_EQ(interfaces[index].get_value(), expected[index]);
  }
}

TEST(WrenchTopicSensorHardware, StaleDataReturnsErrorAndInvalidatesStates)
{
  WrenchTopicSensorHardware hardware;
  WrenchTopicSensorHardwareTestPeer::receive(hardware, 1.0);
  WrenchTopicSensorHardwareTestPeer::expire(hardware);
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
  EXPECT_TRUE(std::isnan(hardware.export_state_interfaces()[0].get_value()));
}

TEST(WrenchTopicSensorHardware, InvalidDataReplacesPreviouslyValidSample)
{
  WrenchTopicSensorHardware hardware;
  WrenchTopicSensorHardwareTestPeer::receive(hardware, 1.0);
  ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::OK);
  WrenchTopicSensorHardwareTestPeer::receive(hardware, std::numeric_limits<double>::infinity());
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
}

TEST(WrenchTopicSensorHardware, UnexpectedFrameReturnsError)
{
  WrenchTopicSensorHardware hardware;
  WrenchTopicSensorHardwareTestPeer::receive(hardware, 1.0, "other_frame");
  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
    hardware_interface::return_type::ERROR);
}

}