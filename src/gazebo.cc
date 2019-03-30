# include "gazebo.hh"

Gazebo::Gazebo(int argc, char* argv[])
  :  node_(new gazebo::transport::Node()),
     ema_filter_{0.9, 42}
{
  gazebo::client::setup(argc, argv);  
  node_->Init();  
}

void Gazebo::subscriber(lt::topic_name name)
{
  if(name == "/gazebo/default/1/2" ) {
  subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_0, this));
  }
  else if (name == "/gazebo/default/1/3") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_1, this));
  }
  else if(name == "/gazebo/default/2/3"){
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_2, this));    
  }
  else if(name == "/gazebo/default/pose/info"){
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_position_msg, this));    
  }  
}

void Gazebo::publisher(lt::topic_name name)
{
  if (name == "/gazebo/default/iris_1/model_reset"){    
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  }
  else if (name == "/gazebo/default/iris_2/model_reset"){    
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  }
  else if (name == "/gazebo/default/iris_3/model_reset"){    
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  }  
}

void Gazebo::reset_models()
{
  for (auto it : pubs_) {
    
    if (it->WaitForConnection(5)) {  
      gazebo::msgs::Vector2d msg;
      gazebo::msgs::Set(&msg, ignition::math::Vector2d(1, 0));  
      it->Publish(msg);
    } else {
      LogDebug() << "NO Connection from the subscriber to reset the model";
    }
  }
}

void Gazebo::Parse_rssi_msg_0(ConstVector2dPtr& msg)
{
  /*  Parsing the RSSI send by NS3 */
  signal_.lf1(msg->x());
}

void Gazebo::Parse_rssi_msg_1(ConstVector2dPtr& msg)
{
  /*  Parsing the RSSI send by NS3 */
  signal_.lf2 (msg->x());
}

void Gazebo::Parse_rssi_msg_2(ConstVector2dPtr& msg)
{
  /*  Parsing the RSSI send by NS3 */
  signal_.ff (msg->x());
}

/*  positin msg received from gazebo */
void Gazebo::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{
  /*  Get the model name from the config ini file */
  
  for (int i =0; i < posesStamped->pose_size(); ++i) {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
    if (name == std::string(config_.quad_names().at(0))) {
      const ::gazebo::msgs::Vector3d& position = pose.position();
      
      positions_.leader.x = position.x();
      positions_.leader.y = position.y();
      positions_.leader.z = position.z();
      
      const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
      
      orientations_.leader.x = orientation.x();
      orientations_.leader.y = orientation.y();
      orientations_.leader.z = orientation.z();
      orientations_.leader.w = orientation.w();
      
    } else if (name == std::string(config_.quad_names().at(1))){
      const ::gazebo::msgs::Vector3d& position = pose.position();
      
      positions_.f1.x = position.x();
      positions_.f1.y = position.y();
      positions_.f1.z = position.z();
      
      
      const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
	
      orientations_.f1.x = orientation.x();
      orientations_.f1.y = orientation.y();
      orientations_.f1.z = orientation.z();
      orientations_.f1.w = orientation.w();		
      
    } else if (name == std::string(config_.quad_names().at(2))) {
      const ::gazebo::msgs::Vector3d& position = pose.position();
      
      positions_.f2.x = position.x();
      positions_.f2.y = position.y();
      positions_.f2.z = position.z();
      
      const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
      
      orientations_.f2.x = orientation.x();
      orientations_.f2.y = orientation.y();
      orientations_.f2.z = orientation.z();
      orientations_.f2.w = orientation.w();	
      
    }
  }
}

lt::rssi<double> Gazebo::rssi() const
{return signal_;}

lt::rssi<double> Gazebo::filtered_rssi() 
{

  ema_filter_.input(signal_.lf1());
  signal_.lf1(ema_filter_.output());
  
  ema_filter_.input(signal_.lf2());
  signal_.lf2(ema_filter_.output());
    
  ema_filter_.input(signal_.ff());
  signal_.ff(ema_filter_.output());
  
  return signal_;
}

positions Gazebo::get_positions() const
{return positions_;}
