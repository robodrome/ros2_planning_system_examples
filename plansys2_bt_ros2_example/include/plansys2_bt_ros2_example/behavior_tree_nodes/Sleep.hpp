// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef plansys2_bt_ros2_example__BEHAVIOR_TREE_NODES__SLEEP_HPP_
#define plansys2_bt_ros2_example__BEHAVIOR_TREE_NODES__SLEEP_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "btcpp_ros2_interfaces/action/sleep.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace plansys2_bt_tests
{

class SleepAction : public BT::RosActionNode<btcpp_ros2_interfaces::action::Sleep>
{
public:
  SleepAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params)
  : RosActionNode<btcpp_ros2_interfaces::action::Sleep>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<unsigned>("msec")});
  }

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

}  // namespace plansys2_bt_tests

#endif  // plansys2_bt_ros2_example__BEHAVIOR_TREE_NODES__SLEEP_HPP_
