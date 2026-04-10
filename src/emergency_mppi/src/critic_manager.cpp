// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "emergency_mppi/critic_manager.hpp"

namespace emergency_mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
  getParam(emergency_critic_names_, "emergency_critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "emergency_mppi", "emergency_mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }

  emergency_critics_.clear();
  for (auto name : emergency_critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    emergency_critics_.push_back(std::move(instance));
    emergency_critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Emergency Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "emergency_mppi::critics::" + name;
}

void CriticManager::setEmergencyMode(bool enabled)
{
  emergency_mode_ = enabled;
}

bool CriticManager::getEmergencyMode() const
{
  return emergency_mode_;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  const auto & active_critics = emergency_mode_ ? emergency_critics_ : critics_;
  const auto & active_critic_names = emergency_mode_ ? emergency_critic_names_ : critic_names_;

  for (size_t i = 0; i < active_critics.size(); ++i) { 
    const auto & critic = active_critics[i]; 
    if (data.fail_flag) {
      break;
    }
    critic->score(data);

    if (data.fail_flag) { 
      RCLCPP_DEBUG( 
        logger_, "Critic '%s' triggered a failure flag.", active_critic_names[i].c_str()); 
    } 
  }
}

}  // namespace emergency_mppi
