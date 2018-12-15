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
  if(name == "/gazebo/default/0/1" ) {
  subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_0, this));
  }
  else if (name == "/gazebo/default/0/2") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_1, this));
  }
  else if(name == "/gazebo/default/1/2"){
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_2, this));    
  }
}

void Gazebo::publisher(lt::topic_name name)
{
  //  PubPtr pub = node_->Advertise<>
}

void Gazebo::reset_world()
{
  
}

void Gazebo::Parse_rssi_msg_0(ConstVector2dPtr& msg)
{
  /*  Parsing the RSSI send by NS3 */
  signal_.lf1 = msg->x();
}

void Gazebo::Parse_rssi_msg_1(ConstVector2dPtr& msg)
{
  /*  Parsing the RSSI send by NS3 */
  signal_.lf2 = msg->x();
}

void Gazebo::Parse_rssi_msg_2(ConstVector2dPtr& msg)
{
  /*  Parsing the RSSI send by NS3 */
  signal_.ff = msg->x();
}


/*  Not used, Good to debug the positin msg */
void Gazebo::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{

  for (int i =0; i < posesStamped->pose_size(); ++i)
    {
      const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
      std::string name = pose.name();
      if (name == std::string("iris"))
        {
          const ::gazebo::msgs::Vector3d& position = pose.position();
	  
          position_.x = position.x();
          position_.y = position.y();
          position_.z = position.z();
	  
	  //std::cout << position_.z << std::endl;
	}
    }
}

lt::rssi<double> Gazebo::rssi() const
{return signal_;}


