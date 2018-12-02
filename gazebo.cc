# include "gazebo.hh"

Gazebo::Gazebo(int argc, char* argv[])
  :  node_(new gazebo::transport::Node())
{
  
  gazebo::client::setup(argc, argv);
  
  node_->Init();
  
}

//template <typename MsgHandler>
void Gazebo::subscriber(lt::topic_name name)
{
  
  SubPtr sub = node_->Subscribe(name, &Gazebo::Parse_position_msg, this);
  subs_.push_back(sub);
  
}

void Gazebo::Parse_rssi_msg()
{



}

void Gazebo::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{

  ::google::protobuf::int32 sec = posesStamped->time().sec();
  ::google::protobuf::int32 nsec = posesStamped->time().nsec();
  //  std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

  for (int i =0; i < posesStamped->pose_size(); ++i)
    {
      const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
      std::string name = pose.name();
      if (name == std::string("iris"))
        {
	  //         std::cout << "ID: " << pose.id() << std::endl;
          const ::gazebo::msgs::Vector3d& position = pose.position();

          position_.x = position.x();
          position_.y = position.y();
          position_.z = position.z();

        //   std::cout << "Read position: x: "
	// 	    << position_.x
        //             << " y: "
	// 	    << position_.y
	// 	    << " z: "
	// 	    << position_.z << std::endl;
	}
    }
  
}

//quad_positions
lt::position Gazebo::get_quads_positions() const
{ return position_;}

//quads_rssi
Eigen::VectorXd Gazebo::get_quads_rssi() const
{return rssi_;}


