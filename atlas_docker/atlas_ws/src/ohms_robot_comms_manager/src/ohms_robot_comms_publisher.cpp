#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"

class OhmsRobotCommsPublisherNode : public rclcpp::Node
{
public:
  OhmsRobotCommsPublisherNode()
    : Node("ohms_robot_comms_publisher_node")
  {
    // Declare and get the robot name parameter
    this->declare_parameter<std::string>("robot_name", "atlas");
    robot_name_ = this->get_parameter("robot_name").as_string();

    // QoS profiles as specified:
    // Tier 1 (Critical - for odometry): Reliable, Transient Local, Depth=10
    rclcpp::QoS tier1_qos(rclcpp::KeepLast(10));
    tier1_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    tier1_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Tier 2 (Perceptual - for octomap): Best Effort, Volatile, Depth=2
    rclcpp::QoS tier2_qos(rclcpp::KeepLast(2));
    tier2_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    tier2_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Internal topics (input)
    std::string internal_odom_topic = "/" + robot_name_ + "/odom_ground_truth";
    std::string internal_binary_oct_topic = "/" + robot_name_ + "/octomap_binary";
    std::string internal_full_oct_topic = "/" + robot_name_ + "/octomap_full";

    // Output (relay) topics with modified QoS
    std::string odom_relay_topic = "/ohms_comm/" + robot_name_ + "/odom";
    std::string binary_oct_relay_topic = "/ohms_comm/" + robot_name_ + "/octomap_binary";
    std::string full_oct_relay_topic = "/ohms_comm/" + robot_name_ + "/octomap_full";

    // Create publishers with required QoS
    odom_relay_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_relay_topic, tier1_qos);
    binary_oct_relay_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(binary_oct_relay_topic, tier2_qos);
    full_oct_relay_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(full_oct_relay_topic, tier2_qos);

    // Subscribers to internal topics
    internal_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      internal_odom_topic, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        // Publish odometry with Tier 1 QoS (reliable, transient local)
        odom_relay_pub_->publish(*msg);
      });

    internal_binary_oct_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      internal_binary_oct_topic, 10,
      [this](octomap_msgs::msg::Octomap::SharedPtr msg) {
        // Publish binary octomap with Tier 2 QoS (best effort, volatile)
        binary_oct_relay_pub_->publish(*msg);
      });

    internal_full_oct_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      internal_full_oct_topic, 10,
      [this](octomap_msgs::msg::Octomap::SharedPtr msg) {
        // Publish full octomap with Tier 2 QoS (best effort, volatile)
        full_oct_relay_pub_->publish(*msg);
      });

    // Log a message indicating successful initialisation
    RCLCPP_INFO(this->get_logger(), "OhmsRobotCommsPublisherNode has started successfully.");
  }

private:
  std::string robot_name_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr internal_odom_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr internal_binary_oct_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr internal_full_oct_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_relay_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_oct_relay_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr full_oct_relay_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OhmsRobotCommsPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
