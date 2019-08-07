#ifndef GAZEBO_HH_
#define GAZEBO_HH_

# include <gazebo/gazebo_config.h>
# include <gazebo/transport/transport.hh>
# include <gazebo/gazebo_client.hh>
# include <gazebo/msgs/msgs.hh>
# include <gazebo/msgs/pose.pb.h>

# include <mutex>

# include "global.hh"
# include "log.hh"
# include "config_ini.hh"

namespace lt = local_types;


class configs;

class Gazebo
{

public:

  using SubPtr                   = gazebo::transport::SubscriberPtr;
  using PubPtr                   = gazebo::transport::PublisherPtr;
  using NodePtr                  = gazebo::transport::NodePtr;

  Gazebo(int argc, char* argv[]);

  void subscriber(lt::topic_name name);

  void publisher(lt::topic_name name);

  void reset_models();

  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);

  void Parse_rssi_msg_0(ConstVector2dPtr& msg);
  void Parse_rssi_msg_1(ConstVector2dPtr& msg);
  void Parse_rssi_msg_2(ConstVector2dPtr& msg);

  lt::rssi<double> rssi() const;
  lt::positions<double> positions() const;
  lt::orientations<double> orientations() const;

  Gazebo(Gazebo const&) = delete;

  Gazebo(Gazebo &&) = default;

private:

  double rssi_;

  std::vector<SubPtr> subs_;

  std::vector<PubPtr> pubs_;

  NodePtr node_;

  mutable std::mutex _positions_mutex{};
  lt::positions<double> _positions;

  mutable std::mutex _orientations_mutex{};
  lt::orientations<double> _orientations;

  mutable std::mutex _signal_mutex{};
  lt::rssi<double> _signal;

  Configs config_;

};

#endif
