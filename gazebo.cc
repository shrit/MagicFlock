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
  else if(name == "/gazebo/default/pose/info"){
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_position_msg, this));    
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


/*  positin msg received from gazebo */
void Gazebo::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{

  for (int i =0; i < posesStamped->pose_size(); ++i)
    {
      const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
      std::string name = pose.name();
      if (name == std::string("iris"))
        {
          const ::gazebo::msgs::Vector3d& position = pose.position();
	  
          positions_.leader.x = position.x();
          positions_.leader.y = position.y();
          positions_.leader.z = position.z();

	  //	  std::cout << "pos" << positions_.leader << std::endl;
	  
	}
      else if (name == std::string("iris_1")){
	const ::gazebo::msgs::Vector3d& position = pose.position();
	
	positions_.f1.x = position.x();
	positions_.f1.y = position.y();
	positions_.f1.z = position.z();
	
	//	std::cout << "pos1" << positions_.f1 << std::endl;
	
      }
      else if (name == std::string("iris_2")){
	const ::gazebo::msgs::Vector3d& position = pose.position();
	
	positions_.f2.x = position.x();
	positions_.f2.y = position.y();
	positions_.f2.z = position.z();
	
	//std::cout << "pos2" << positions_.f2 << std::endl;
	
      }
    }
}

lt::rssi<double> Gazebo::rssi() const
{return signal_;}

positions Gazebo::get_positions() const
{return positions_;}
