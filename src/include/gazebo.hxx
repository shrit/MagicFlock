#include "../include/gazebo.hh"

template<class QuadrotorType>
Gazebo<QuadrotorType>::Gazebo(int argc, char* argv[])
  : node_(new gazebo::transport::Node())
  , _positions(4, ignition::math::Vector3d())
  , _orientations(4, ignition::math::Quaternion<double>())
{
  gazebo::client::setup(argc, argv);
  node_->Init();
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::subsPosTimeTopic()
{
  subs_.push_back(node_->Subscribe(
    "/gazebo/default/pose/info", &Gazebo<QuadrotorType>::PosMsg, this));
  subs_.push_back(node_->Subscribe(
    "/gazebo/default/world_stats", &Gazebo<QuadrotorType>::TimeMsg, this));
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::subRxTopic()
{
  for (auto it : quadrotors) {
    std::string topic_WR_1 =
      "/gazebo/default/" + it->name() + "WR_1/Wireless Reveiver/transceiver";
    subs_.push_back(
      node_->Subscribe(topic_WR_1, &Gazebo<QuadrotorType>::RxMsg, this));
    std::string topic_WR_2 =
      "/gazebo/default/" + it->name() + "WR_2/Wireless Reveiver/transceiver";
    subs_.push_back(
      node_->Subscribe(topic_WR_2, &Gazebo<QuadrotorType>::RxMsg, this));
  }
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::pubModelReset()
{
  for (auto it : quadrotors) {
    std::string topic = "/gazebo/default/" + it->name() + "/model_reset";
    pubs_.push_back(node_->Advertise<gazebo::msgs::Vector2d>(topic));
  }
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::ResetModels()
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

/*  Parsing the RSSI send by Gazebo */
template<class QuadrotorType>
void
Gazebo::RxMsg(const ConstWirelessNodesPtr& _msg)
{
  std::lock_guard<std::mutex> lock(_rx_mutex);
  this->RxNodesMsg = _msg;
  gazebo::msgs::WirelessNodes txNodes;
  int numTxNodes = nodesMsg->node_size();

  for (int i = 0; i < numTxNodes; ++i) {
    gazebo::msgs::WirelessNode RxNode = RxNodesMsg->node(i);
    std::string essid = txNode.essid();
    txNode.frequency();
    txNode.signal_level();
  }
}

/*  Position messages received from gazebo topics */
template<class QuadrotorType>
void
Gazebo<QuadrotorType>::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{
  /*  Get the model name from the config ini file */
  for (int i = 0; i < posesStamped->pose_size(); ++i) {
    const ::gazebo::msgs::Pose& pose = posesStamped->pose(i);
    std::string name = pose.name();
    for (std::size_t j = 0; j < config_.quad_names().size(); ++j) {
      if (name == std::string(config_.quad_names().at(j))) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        _positions.at(j) = ::gazebo::msgs::ConvertIgn(position);

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
        _orientations.at(j) = ::gazebo::msgs::ConvertIgn(orientation);

      } else if (name = std::string(config_.WT_names().at(j))) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        _WT_positions.at(j) = ::gazebo::msgs::ConvertIgn(position);

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
        _WT_orientations.at(j) = ::gazebo::msgs::ConvertIgn(orientation);

      } else if (name == std::string(config_.WR_names().at(j))) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        _WR_positions.at(j) = ::gazebo::msgs::ConvertIgn(position);

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
        _WR_orientations.at(j) = ::gazebo::msgs::ConvertIgn(orientation);
      }
    }
  }
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::Parse_time_msg(ConstWorldStatisticsPtr& msg)
{
  Time t(msg->sim_time().sec(), msg->sim_time().nsec());
  Time rt(msg->real_time().sec(), msg->real_time().nsec());
  Time::setTime(t, t.seconds() / rt.seconds());
}

// template<class QuadrotorType>
// void
// Gazebo<QuadrotorType>::spawn(const std::vector<ignition::math::Vector3d>&
// homes,
//               std::string sdf_file,
//               std::string rcs_file)
// {
// tansa::msgs::SpawnRequest req;

// for (int i = 0; i < homes.size(); i++) {
//   tansa::msgs::SpawnRequest_Vehicle* v = req.add_vehicles();
//   v->set_id(i);
//   gazebo::msgs::Vector3d* pos = v->mutable_pos();
//   gazebo::msgs::Vector3d* orient = v->mutable_orient();
//   pos->set_x(homes[i].x());
//   pos->set_y(homes[i].y());
//   pos->set_z(homes[i].z());

//   orient->set_x(0);
//   orient->set_y(0);
//   orient->set_z(0);
// }

// req.set_sdf_file(sdf_file);
// req.set_rcs_file(rcs_file);
// spawn_pub->Publish(req);
// }

