#include <string>
#include <memory>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief BT::ConditionNode that checks if dynamic threats (/scan_threat) are present.
 * Returns SUCCESS if threat points >= min_points, FAILURE otherwise.
 */
class IsThreatPresentCondition : public BT::ConditionNode
{
public:
  IsThreatPresentCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    threat_topic_("/scan_threat"),
    min_points_(3),
    is_threat_present_(false),
    last_msg_time_(0.0)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    getInput("topic", threat_topic_);
    getInput("min_points", min_points_);

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    threat_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      threat_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&IsThreatPresentCondition::scanCallback, this, std::placeholders::_1),
      sub_option);
  }

  IsThreatPresentCondition() = delete;

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();

    // 마지막 수신으로부터 1.0초 이상 지나면 위협 없음으로 간주
    double now_sec = node_->now().seconds();
    if (now_sec - last_msg_time_ > 1.0) {
      is_threat_present_ = false;
    }

    if (is_threat_present_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", std::string("/scan_threat"), "Dynamic threat scan topic"),
      BT::InputPort<int>("min_points", 3, "Minimum points to consider as threat"),
    };
  }

private:
  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int count = 0;
    for (const auto & r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        count++;
      }
    }
    is_threat_present_ = (count >= min_points_);
    last_msg_time_ = node_->now().seconds();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr threat_sub_;
  std::string threat_topic_;
  int min_points_;
  bool is_threat_present_;
  double last_msg_time_;
};

}  // namespace nav2_behavior_tree

#ifndef BT_PLUGIN_EXPORT
#define BT_PLUGIN_EXPORT
#endif
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsThreatPresentCondition>("IsThreatPresent");
}
