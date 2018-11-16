#ifndef GAZEBO_HH_
#define GAZEBO_HH_

# include <gazebo/gazebo_config.h>
# include <gazebo/transport/transport.hh>
# include <gazebo/gazebo_client.hh>
# include <gazebo/msgs/msgs.hh>
# include <gazebo/msgs/pose.pb.h>
# include <Eigen/Dense>

# include "global.hh"

namespace lt = local_types;

class Gazebo
{

  using SubPtr                   = gazebo::transport::SubscriberPtr;
  using NodePtr                  = gazebo::transport::NodePtr;
  using quads_rssi               = Eigne::VectorXd;
  
public:

  Gazebo(int argc, char* argv[]);
  /*  This function subscribe to any topic and handle 
   the message */
  //  template <typename MsgHandler> , MsgHandler&& handler // to see later
  void subscriber(lt::topic_name name);
  //  quad_positions
  lt::position get_quads_positions() const;

  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);
  
  //  quads_rssi
  quads_rssi get_quads_rssi() const;
  
  
private:

  //  signal_type parse_msg;
  /*  node process that connecte to gazebo topic */
  NodePtr node_;
  
  lt::position position_;

  quads_rssi     rssi_;
  
  std::vector<SubPtr> subs_;
  
  
};


#endif 