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


struct positions {
  lt::position<double> leader;
  lt::position<double> f1;
  lt::position<double> f2;
};


class Gazebo
{

public:

  using SubPtr                   = gazebo::transport::SubscriberPtr;
  using PubPtr                   = gazebo::transport::PublisherPtr;
  using NodePtr                  = gazebo::transport::NodePtr;
  
  Gazebo(int argc, char* argv[]);

  /*  This function subscribe to any topic and handle 
   the message */
  //  template <typename MsgHandler> , MsgHandler&& handler // to see later

  void subscriber(lt::topic_name name);
  
  void publisher(lt::topic_name name);

  void reset_world();

  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);

  void Parse_rssi_msg_0(ConstVector2dPtr& msg);
  void Parse_rssi_msg_1(ConstVector2dPtr& msg);
  void Parse_rssi_msg_2(ConstVector2dPtr& msg);
  
  lt::rssi<double> rssi() const;
  positions get_positions() const;
  
private:

  //  signal_type parse_msg;
  /*  node process that connecte to gazebo topic */
  double rssi_;

  std::vector<SubPtr> subs_;
  
  NodePtr node_;
  
  positions positions_;
  
  lt::rssi<double> signal_;   
     
};


#endif 
