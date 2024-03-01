#include "plansys2_bt_ros2_example/behavior_tree_nodes/Sleep.hpp"

namespace plansys2_bt_tests
{

bool SleepAction::setGoal(RosActionNode::Goal & goal)
{
  auto timeout = getInput<unsigned>("msec");
  goal.msec_timeout = timeout.value();
  RCLCPP_INFO(
    node_->get_logger(), "%s: setGoal. msec_timeout = %d", name().c_str(), goal.msec_timeout);
  return true;
}

BT::NodeStatus SleepAction::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(
    node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(),
    wr.result->done ? "true" : "false");

  return wr.result->done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SleepAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

void SleepAction::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "%s: onHalt", name().c_str() );
}


}  // namespace plansys2_bt_tests

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(plansys2_bt_tests::SleepAction, "SleepAction");
