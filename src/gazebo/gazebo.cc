#include "include/gazebo.hh"

Gazebo::Gazebo(int argc, char* argv[], Configs config)
  : node_(new gazebo::transport::Node())
  , _positions(4, lt::position3D<double>())
  , _orientations(4, lt::orientation<double>())
  , config_(config)
{
  gazebo::client::setup(argc, argv);
  node_->Init();
}

void
Gazebo::subscriber(lt::topic_name name)
{
  if (name == "/gazebo/default/1/2") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_0, this));
  } else if (name == "/gazebo/default/1/3") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_1, this));
  } else if (name == "/gazebo/default/2/3") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_rssi_msg_2, this));
  } else if (name == "/gazebo/default/pose/info") {
    subs_.push_back(node_->Subscribe(name, &Gazebo::Parse_position_msg, this));
  } else if (name == "/gazebo/default/world_stats")
    subs_.push_back(node->Subscribe(name, &Gazebo::Parse_time_msg, this));
}

void
Gazebo::publisher(lt::topic_name name)
{
  if (name == "/gazebo/default/iris_1/model_reset") {
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  } else if (name == "/gazebo/default/iris_2/model_reset") {
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  } else if (name == "/gazebo/default/iris_3/model_reset") {
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  } else if (name == "/gazebo/default/iris_4/model_reset") {
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(name));
  }
}

void
Gazebo::reset_models()
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
void
Gazebo::Parse_rssi_msg_0(ConstVector2dPtr& msg)
{
  _signal.f3(msg->x());
}

/*  Parsing the RSSI send by NS3 */
void
Gazebo::Parse_rssi_msg_1(ConstVector2dPtr& msg)
{
  _signal.f1(msg->x());
}

/*  Parsing the RSSI send by NS3 */
void
Gazebo::Parse_rssi_msg_2(ConstVector2dPtr& msg)
{
  _signal.f2(msg->x());
}

/*  positin msg received from gazebo */
void
Gazebo::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{
  /*  Get the model name from the config ini file */
  for (int i = 0; i < posesStamped->pose_size(); ++i) {
    const ::gazebo::msgs::Pose& pose = posesStamped->pose(i);
    std::string name = pose.name();
    for (std::size_t j = 0; j < config_.quad_names().size(); ++j) {
      if (name == std::string(config_.quad_names().at(j))) {
        const ::gazebo::msgs::Vector3d& position = pose.position();

        _positions.at(j).x = position.x();
        _positions.at(j).y = position.y();
        _positions.at(j).z = position.z();

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();

        _orientations.at(j).x = orientation.x();
        _orientations.at(j).y = orientation.y();
        _orientations.at(j).z = orientation.z();
        _orientations.at(j).w = orientation.w();
      }
    }
  }
}

void
Gazebo::Parse_time_msg(ConstWorldStatisticsPtr& msg)
{
  msg->sim_time().sec(), msg->sim_time().nsec());
 Time rt(msg->real_time().sec(), msg->real_time().nsec());

}

void
GazeboConnector::spawn(const std::vector<Point>& homes,
                       std::string sdf_file,
                       std::string rcs_file)
{
  tansa::msgs::SpawnRequest req;

  for (int i = 0; i < homes.size(); i++) {
    tansa::msgs::SpawnRequest_Vehicle* v = req.add_vehicles();
    v->set_id(i);
    gazebo::msgs::Vector3d* pos = v->mutable_pos();
    gazebo::msgs::Vector3d* orient = v->mutable_orient();
    pos->set_x(homes[i].x());
    pos->set_y(homes[i].y());
    pos->set_z(homes[i].z());

    orient->set_x(0);
    orient->set_y(0);
    orient->set_z(0);
  }

  req.set_sdf_file(sdf_file);
  req.set_rcs_file(rcs_file);
  spawn_pub->Publish(req);
}

lt::rssi<double>
Gazebo::rssi() const
{
  std::lock_guard<std::mutex> lock(_signal_mutex);
  return _signal;
}

std::vector<lt::position3D<double>>
Gazebo::positions() const
{
  std::lock_guard<std::mutex> lock(_positions_mutex);
  return _positions;
}

std::vector<lt::orientation<double>>
Gazebo::orientations() const
{
  std::lock_guard<std::mutex> lock(_orientations_mutex);
  return _orientations;
}
