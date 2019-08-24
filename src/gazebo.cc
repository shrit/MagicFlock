# include "gazebo.hh"

Gazebo::Gazebo(int argc, char* argv[])
  : node_(new gazebo::transport::Node())
{
  gazebo::client::setup(argc, argv);
  node_->Init();
}

void Gazebo::subscriber(lt::topic_name name)
{
  if (name == "/gazebo/default/1/2" ) {
  subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_0, this));
  } else if (name == "/gazebo/default/1/3") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_1, this));
  } else if (name == "/gazebo/default/2/3") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_2, this));
  } else if (name == "/gazebo/default/pose/info") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_position_msg, this));
  }
}

void Gazebo::publisher(lt::topic_name name)
{
  if (name == "/gazebo/default/iris_1/model_reset") {
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  } else if (name == "/gazebo/default/iris_2/model_reset") {
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  } else if (name == "/gazebo/default/iris_3/model_reset") {
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

  /*  Parsing the RSSI send by NS3 */
void Gazebo::Parse_rssi_msg_0(ConstVector2dPtr& msg)
{
  _signal.f3(msg->x());
}

  /*  Parsing the RSSI send by NS3 */
void Gazebo::Parse_rssi_msg_1(ConstVector2dPtr& msg)
{
  _signal.f1 (msg->x());
}

/*  Parsing the RSSI send by NS3 */
void Gazebo::Parse_rssi_msg_2(ConstVector2dPtr& msg)
{
  _signal.f2 (msg->x());
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

      _positions.leader.x = position.x();
      _positions.leader.y = position.y();
      _positions.leader.z = position.z();

      const ::gazebo::msgs::Quaternion& orientation = pose.orientation();

      _orientations.leader.x = orientation.x();
      _orientations.leader.y = orientation.y();
      _orientations.leader.z = orientation.z();
      _orientations.leader.w = orientation.w();

    } else if (name == std::string(config_.quad_names().at(1))) {
      const ::gazebo::msgs::Vector3d& position = pose.position();

      _positions.f1.x = position.x();
      _positions.f1.y = position.y();
      _positions.f1.z = position.z();


      const ::gazebo::msgs::Quaternion& orientation = pose.orientation();

      _orientations.f1.x = orientation.x();
      _orientations.f1.y = orientation.y();
      _orientations.f1.z = orientation.z();
      _orientations.f1.w = orientation.w();

    } else if (name == std::string(config_.quad_names().at(2))) {
      const ::gazebo::msgs::Vector3d& position = pose.position();

      _positions.f2.x = position.x();
      _positions.f2.y = position.y();
      _positions.f2.z = position.z();

      const ::gazebo::msgs::Quaternion& orientation = pose.orientation();

      _orientations.f2.x = orientation.x();
      _orientations.f2.y = orientation.y();
      _orientations.f2.z = orientation.z();
      _orientations.f2.w = orientation.w();
    }
  }
}

lt::rssi<double> Gazebo::rssi() const
{
  std::lock_guard<std::mutex> lock(_signal_mutex);
  return _signal;
}

lt::positions<lt::position3D<double>> Gazebo::positions() const
{
  std::lock_guard<std::mutex> lock(_positions_mutex);
  return _positions;
}

lt::orientations<double> Gazebo::orientations() const
{
  std::lock_guard<std::mutex> lock(_orientations_mutex);
  return _orientations;
}
