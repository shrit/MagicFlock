# include <gazebo/gazebo_config.h>
# include <gazebo/transport/transport.hh>
# include <gazebo/gazebo_client.hh>
# include <gazebo/msgs/msgs.hh>
# include <gazebo/msgs/pose.pb.h>


struct position {

  double x;
  double y;
  double z;
  
};

class Gazebo
{

  using SubPtr                   = gazebo::transport::SubscriberPtr;
  using NodePtr                  = gazebo::transport::NodePtr;
  using topic_name               = std::string;
  using quad_positions           = gazebo::msgs::Vector3d;
  using quads_rssi               = gazebo::msgs::Vector3d;

  //  using signal_type              = boost::signals2::signal<void (const ConstPosesStampedPtr&)> ;

  
public:

  Gazebo(int argc, char* argv[]);
  /*  This function subscribe to any topic and handle 
   the message */
  //  template <typename MsgHandler> , MsgHandler&& handler // to see later
  void subscriber(topic_name name);
  //  quad_positions
  position get_quads_positions() const;

  void Parse_position_msg(ConstPosesStampedPtr& posesStamped);
  
  //  quads_rssi
   gazebo::msgs::Vector3d get_quads_rssi() const;
  
  
private:

  //  signal_type parse_msg;
  /*  node process that connecte to gazebo topic */
  NodePtr node_;
  
  position position_;

  quads_rssi     rssi_;
  
  std::vector<SubPtr> subs_;
  
  
};
