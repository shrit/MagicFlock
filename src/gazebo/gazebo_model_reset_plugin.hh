/*
 * @brief Model reset plugin
 *
 * This is a reset plugin to bring the quadrotors back at the end of each
 * episode.
 * It is possible to bring the quadrotors either to their initial positions, or
 * to a random position with a specific place as a starting point.
 * A random generator will genenrate the cooridnation of the new position with
 * respect to other quadrotors positions. A radius is defined as the max
 * distance between the quadrotrs and a min distance to avoid collision during
 * the spawning.
 *
 * Some part of this code has been inspired by tansa the dancing swarm
 * platforme.
 *
 * @author: Omar Shrit <shrit@lri.fr>
 * */

#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include "ResetModel.pb.h"

#include <vector>
#include <random>
#include <utility>

using ConstResetModelPtr =
  const boost::shared_ptr<const reset_model_msg::msg::ResetModel>;

namespace gazebo {
class ResetPlugin : public WorldPlugin
{
public:
  ResetPlugin();

  void Load(physics::WorldPtr _parent, sdf::ElementPtr);
  void OnMsg(ConstResetModelPtr& _msg);
  std::vector<ignition::math::Vector3d> RandomPoseGenerator(int quad_number);

private:
  physics::ModelPtr model;
  physics::WorldPtr world_;
  transport::NodePtr node;
  transport::SubscriberPtr sub;

  std::uniform_real_distribution<> distribution_real_;
  std::random_device random_dev;
  std::mt19937 generator_;
};
}
