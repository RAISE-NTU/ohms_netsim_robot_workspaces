GNU nano 6.2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          /atlas_ws/src/ohms_robot_comms_manager/src/ohms_robot_comms_subscriber.cpp                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <deque>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/subscription_options.hpp"


struct RobotConnectionInfo
{
  double rx_strength;
  double bandwidth;
  double delay;
  double link_quality;

  // Sliding window for link quality
  std::deque<double> link_quality_history;

  // Input Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rx_strength_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bandwidth_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr delay_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr binary_oct_sub;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr full_oct_sub;

  // Topics needed for (re-)subscriptions
  std::string binary_oct_topic;
  std::string full_oct_topic;

  bool full_oct_subscribed = false;

  // Publishers for processed data
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr link_quality_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_oct_pub;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr full_oct_pub;
};

class OhmsRobotCommsSubscriberNode : public rclcpp::Node
{
public:
  OhmsRobotCommsSubscriberNode()
  : Node("ohms_robot_comms_subscriber_node"),
    this_robot_name_("atlas"),
    link_quality_threshold_(5.0),
    window_size_(10) // Default window size
  {
    // Define QoS profiles
    rclcpp::QoS tier1_qos(rclcpp::KeepLast(10));
    tier1_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    tier1_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rclcpp::QoS tier2_qos(rclcpp::KeepLast(2));
    tier2_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    tier2_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Declare and get parameters
    this->declare_parameter<std::vector<std::string>>("other_robots", std::vector<std::string>{"bestla"});
    other_robots_ = this->get_parameter("other_robots").as_string_array();

    this->declare_parameter<double>("link_quality_threshold", 5.0);
    link_quality_threshold_ = this->get_parameter("link_quality_threshold").as_double();

    this->declare_parameter<int>("window_size", 10);
    window_size_ = this->get_parameter("window_size").as_int();

    // For each other robot, create subscriptions and publishers
    for (const auto &other_robot : other_robots_) {
      std::string robot_pair = other_robot + "_to_" + this_robot_name_;

      RobotConnectionInfo info;
      info.rx_strength = 100.0; // initial assumption
      info.bandwidth = 0.0;
      info.delay = 0.0;
      info.link_quality = 100.0;

      // Topics for this pair
      std::string ss_topic = "/robot_comms_emu_helper/" + robot_pair + "/rx_strength";
      std::string bw_topic = "/robot_comms_emu_helper/" + robot_pair + "/bandwidth";
      std::string dly_topic = "/robot_comms_emu_helper/" + robot_pair + "/delay";

      std::string odom_topic = "/ohms_comm/" + other_robot + "/odom";
      info.binary_oct_topic = "/ohms_comm/" + other_robot + "/octomap_binary";
      info.full_oct_topic = "/ohms_comm/" + other_robot + "/octomap_full";

      std::string lq_topic = "/" + this_robot_name_ + "/" + robot_pair + "/link_quality";

      // Output (republished) topics
      std::string output_odom_topic = "/" + this_robot_name_ + "/" + robot_pair + "/odom";
      std::string output_binary_oct_topic = "/" + this_robot_name_ + "/" + robot_pair + "/octomap_binary";
      std::string output_full_oct_topic = "/" + this_robot_name_ + "/" + robot_pair + "/octomap_full";

      // Store info in map before subscriptions
      connections_[other_robot] = info;

      // Create input subscriptions for link metrics
      connections_[other_robot].rx_strength_sub = this->create_subscription<std_msgs::msg::Float64>(
        ss_topic, 10,
        [this, other_robot](std_msgs::msg::Float64::SharedPtr msg) {
          connections_[other_robot].rx_strength = msg->data;
          updateLinkQuality(other_robot);
        }
      );

      connections_[other_robot].bandwidth_sub = this->create_subscription<std_msgs::msg::Float64>(
        bw_topic, 10,
        [this, other_robot](std_msgs::msg::Float64::SharedPtr msg) {
          connections_[other_robot].bandwidth = msg->data;
          updateLinkQuality(other_robot);
        }
      );

      connections_[other_robot].delay_sub = this->create_subscription<std_msgs::msg::Float64>(
        dly_topic, 10,
        [this, other_robot](std_msgs::msg::Float64::SharedPtr msg) {
          connections_[other_robot].delay = msg->data;
          updateLinkQuality(other_robot);
        }
      );

      // -------------------------------------------------
      // Create SubscriptionOptions for Topic Statistics
      // -------------------------------------------------
      rclcpp::SubscriptionOptions odom_stats_options;
      odom_stats_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
      // Publish statistics every 1 second
      odom_stats_options.topic_stats_options.publish_period = std::chrono::seconds(1);
      // Override the default topic name for stats:
      odom_stats_options.topic_stats_options.publish_topic = "/" + robot_pair + "/odom_topic_statistics";

      // Create odom subscription and publisher (Tier 1 QOS)
      connections_[other_robot].odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, tier1_qos,
        [this, other_robot](nav_msgs::msg::Odometry::SharedPtr msg) {
          //RCLCPP_INFO(this->get_logger(), "[%s->%s] Processing odom",
          //            other_robot.c_str(), this_robot_name_.c_str());
          connections_[other_robot].odom_pub->publish(*msg);
        },
        odom_stats_options
      );

      connections_[other_robot].odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic, tier1_qos);

      // -------------------------------------------------
      // Create SubscriptionOptions for Topic Statistics
      // -------------------------------------------------
      rclcpp::SubscriptionOptions bin_ocmap_stats_options;
      bin_ocmap_stats_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
      // Publish statistics every 1 second
      bin_ocmap_stats_options.topic_stats_options.publish_period = std::chrono::seconds(1);
      // Override the default topic name for stats:
      bin_ocmap_stats_options.topic_stats_options.publish_topic = "/" + robot_pair + "/bin_ocmap_topic_statistics";

      // Always subscribe to binary octomap (Tier 2 QOS)
      connections_[other_robot].binary_oct_sub = this->create_subscription<octomap_msgs::msg::Octomap>(
        connections_[other_robot].binary_oct_topic, tier2_qos,
        [this, other_robot](octomap_msgs::msg::Octomap::SharedPtr msg) {
          //RCLCPP_INFO(this->get_logger(), "[%s->%s] Processing binary octomap",
          //            other_robot.c_str(), this_robot_name_.c_str());
          connections_[other_robot].binary_oct_pub->publish(*msg);
        },
        bin_ocmap_stats_options
      );

      connections_[other_robot].binary_oct_pub = this->create_publisher<octomap_msgs::msg::Octomap>(output_binary_oct_topic, tier2_qos);

      // Initially, not subscribed to full octomap
      connections_[other_robot].full_oct_sub = nullptr;
      connections_[other_robot].full_oct_subscribed = false;

      // Full octomap publisher (Tier 2 QOS)
      connections_[other_robot].full_oct_pub = this->create_publisher<octomap_msgs::msg::Octomap>(output_full_oct_topic, tier2_qos);

      // Link quality publisher
      connections_[other_robot].link_quality_pub = this->create_publisher<std_msgs::msg::Float64>(lq_topic, 10);
    }

    // Timer to periodically log link quality
    timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
      for (const auto &pair : connections_) {
        const auto &robot = pair.first;
        const auto &info = pair.second;
        RCLCPP_INFO(this->get_logger(),
                    "[%s->%s] Current link quality: %.2f (rx_strength=%.2f, bandwidth=%.2f, delay=%.2f)",
                    robot.c_str(), this_robot_name_.c_str(), info.link_quality,
                    info.rx_strength, info.bandwidth, info.delay);
      }
    });
  }

private:
  void updateLinkQuality(const std::string &other_robot)
  {
    auto &info = connections_.at(other_robot);
    double current_lq = (info.rx_strength + info.bandwidth - info.delay) / 3.0;

    // Update sliding window
    if (info.link_quality_history.size() >= static_cast<size_t>(window_size_)) {
      info.link_quality_history.pop_front();
    }
    info.link_quality_history.push_back(current_lq);

    // Calculate mean link quality
    double sum = 0.0;
    for (const auto &val : info.link_quality_history) {
      sum += val;
    }
    double mean_lq = sum / info.link_quality_history.size();
    double old_lq = info.link_quality;
    info.link_quality = mean_lq;

    // Publish link quality
    std_msgs::msg::Float64 lq_msg;
    lq_msg.data = info.link_quality;
    info.link_quality_pub->publish(lq_msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "[%s->%s] Link quality updated: %.2f (old=%.2f, current=%.2f, rx_strength=%.2f, bandwidth=%.2f, delay=%.2f)",
                 other_robot.c_str(), this_robot_name_.c_str(),
                 info.link_quality, old_lq, current_lq, info.rx_strength, info.bandwidth, info.delay);

    // Determine hysteresis thresholds
    bool should_subscribe_full = (info.link_quality >= link_quality_threshold_ + 3.0);
    bool should_unsubscribe_full = (info.link_quality <= link_quality_threshold_ - 3.0);

    if (should_subscribe_full && !info.full_oct_subscribed) 
    {
      RCLCPP_INFO(this->get_logger(), "[%s->%s] Subscribing to full octomap",
                  other_robot.c_str(), this_robot_name_.c_str());
      // Subscribe to full octomap (Tier 2 QoS)
      rclcpp::QoS tier2_qos(rclcpp::KeepLast(2));
      tier2_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      tier2_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      // Re-use topic statistics settings
      rclcpp::SubscriptionOptions full_ocmap_stats_options;
      full_ocmap_stats_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
      full_ocmap_stats_options.topic_stats_options.publish_period = std::chrono::seconds(1);

      info.full_oct_sub = this->create_subscription<octomap_msgs::msg::Octomap>(
        info.full_oct_topic, tier2_qos,
        [this, other_robot](octomap_msgs::msg::Octomap::SharedPtr msg) {
          if (connections_[other_robot].link_quality >= link_quality_threshold_) {
            //RCLCPP_INFO(this->get_logger(), "[%s->%s] Processing full octomap",
            //            other_robot.c_str(), this_robot_name_.c_str());
            connections_[other_robot].full_oct_pub->publish(*msg);
          }
        },
        full_ocmap_stats_options
      );
      info.full_oct_subscribed = true;
    }
    else if (should_unsubscribe_full && info.full_oct_subscribed)
    {
      RCLCPP_INFO(this->get_logger(), "[%s->%s] Unsubscribing from full octomap",
                  other_robot.c_str(), this_robot_name_.c_str());
      info.full_oct_sub.reset();
      info.full_oct_subscribed = false;
    }
  }

  std::string this_robot_name_;
  double link_quality_threshold_;
  int window_size_;
  std::vector<std::string> other_robots_;
  std::map<std::string, RobotConnectionInfo> connections_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OhmsRobotCommsSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



