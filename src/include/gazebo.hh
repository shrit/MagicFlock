#pragma once

#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/transport/transport.hh>
#include <ignition/math4/ignition/math/Quaternion.hh>
#include <ignition/math6/ignition/math/Vector2.hh>
#include <ignition/math6/ignition/math/Vector3.hh>

#include <mutex>
#include <vector>

#include "time.hh"

template<class QuadrotorType>
class Gazebo
{

public:
  using SubPtr = gazebo::transport::SubscriberPtr;
  using PubPtr = gazebo::transport::PublisherPtr;
  using NodePtr = gazebo::transport::NodePtr;

  Gazebo(int argc,
         char* argv[],
         std::vector<std::shared_ptr<QuadrotorType>>& quadrotors);

  void subscribe_position_topic();
  void publishe_model_reset(std::string name);
  void reset_models();

  void Parse_time_msg(ConstWorldStatisticsPtr& msg);
  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);
  void Parse_rssi_msg(ConstVector2dPtr& msg);

  // void spawn(const std::vector<ignition::math::Vector3d>& homes,
  //            std::string sdf_file,
  //            std::string rcs_file);

  ignition::math::Vector2d rssi() const;

  Gazebo(Gazebo const&) = delete;
  Gazebo(Gazebo&&) = default;

private:
  std::vector<SubPtr> subs_;
  std::vector<PubPtr> pubs_;
  NodePtr node_;

  mutable std::mutex _signal_mutex{};
  ignition::math::Vector2d _signal;
};
