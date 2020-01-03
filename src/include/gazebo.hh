#ifndef GAZEBO_HH_
#define GAZEBO_HH_

#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/transport/transport.hh>

#include <mutex>

#include "config_ini.hh"
#include "global.hh"
#include "log.hh"

namespace lt = local_types;

class Gazebo
{
public:
  using SubPtr = gazebo::transport::SubscriberPtr;
  using PubPtr = gazebo::transport::PublisherPtr;
  using NodePtr = gazebo::transport::NodePtr;

  Gazebo(int argc, char* argv[], Configs config);

  void subscriber(lt::topic_name name);
  void publisher(lt::topic_name name);
  void reset_models();

  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);
  void Parse_rssi_msg_0(ConstVector2dPtr& msg);
  void Parse_rssi_msg_1(ConstVector2dPtr& msg);
  void Parse_rssi_msg_2(ConstVector2dPtr& msg);

  lt::rssi<double> rssi() const;
  std::vector<lt::position3D<double>> positions() const;
  std::vector<lt::orientation<double>> orientations() const;

  Gazebo(Gazebo const&) = delete;
  Gazebo(Gazebo&&) = default;

private:
  double rssi_;
  std::vector<SubPtr> subs_;
  std::vector<PubPtr> pubs_;
  NodePtr node_;

  mutable std::mutex _positions_mutex{};
  std::vector<lt::position3D<double>> _positions;

  mutable std::mutex _orientations_mutex{};
  std::vector<lt::orientation<double>> _orientations;

  mutable std::mutex _signal_mutex{};
  lt::rssi<double> _signal;

  Configs config_;
};

#endif
