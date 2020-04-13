#ifndef GAZEBO_HH_
#define GAZEBO_HH_

#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/transport/transport.hh>
#include <ignition/math6/ignition/math/Vector2.hh>
#include <ignition/math6/ignition/math/Vector3.hh>
#include <ignition/math6/ignition/math/Vector4.hh>

#include <mutex>
#include <vector>

#include "config_ini.hh"
#include "log.hh"

class Gazebo
{
  using topic_name = std::string;

public:
  using SubPtr = gazebo::transport::SubscriberPtr;
  using PubPtr = gazebo::transport::PublisherPtr;
  using NodePtr = gazebo::transport::NodePtr;

  Gazebo(int argc, char* argv[], Configs config);

  void subscriber(topic_name name);
  void publisher(topic_name name);
  void reset_models();

  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);
  void Parse_rssi_msg_0(ConstVector2dPtr& msg);
  void Parse_rssi_msg_1(ConstVector2dPtr& msg);
  void Parse_rssi_msg_2(ConstVector2dPtr& msg);

  void spawn(const std::vector<Point>& homes,
             std::string sdf_file,
             std::string rcs_file);

  ignition::math::Vector2d rssi() const;
  std::vector<ignition::math::Vector3d> positions() const;
  std::vector<ignition::math::Vector4d> orientations() const;

  Gazebo(Gazebo const&) = delete;
  Gazebo(Gazebo&&) = default;

private:
  double rssi_;
  std::vector<SubPtr> subs_;
  std::vector<PubPtr> pubs_;
  NodePtr node_;

  mutable std::mutex _positions_mutex{};
  std::vector<ignition::math::Vector3d> _positions;

  mutable std::mutex _orientations_mutex{};
  std::vector<ignition::math::Vector4d> _orientations;

  mutable std::mutex _signal_mutex{};
  ignition::math::Vector2d _signal;

  Configs config_;
};

#endif
