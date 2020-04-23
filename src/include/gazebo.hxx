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
    std::string topic_WR_1 = it->wireless_receiver_1_topic_name();
    subs_.push_back(
      node_->Subscribe(topic_WR_1, &Gazebo<QuadrotorType>::RxMsg, this));
    std::string topic_WR_2 = it->wireless_receiver_2_topic_name();
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
      reset_model_msg::msg::ResetModel msg;
      gazebo::msgs::Set(&msg, true);
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
  this->_RxNodesMsg = _msg;
  gazebo::msgs::WirelessNodes txNodes;
  int numTxNodes = nodesMsg->node_size();

  for (int i = 0; i < numTxNodes; ++i) {
    gazebo::msgs::WirelessNode RxNode = _RxNodesMsg->node(i);
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
  for (int i = 0; i < posesStamped->pose_size(); ++i) {
    const ::gazebo::msgs::Pose& pose = posesStamped->pose(i);
    std::string name = pose.name();
    for (std::size_t j = 0; j < quadrotors.size(); ++j) {
      if (name == std::string(quadrotors.at(j)->names()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors.at(j)->position() = ::gazebo::msgs::ConvertIgn(position);

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
        quadrotors.at(j)->orientation() =
          ::gazebo::msgs::ConvertIgn(orientation);

      } else if (name = std::string(quadrotors.at(j)->wt_name()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors.at(j)->wt_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);

      } else if (name == std::string(quadrotors.at(j)->wr_1_name()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors.at(j)->wr_1_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);

      } else if (name == std::string(quadrotors.at(j)->wr_2_name()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors.at(j)->wr_2_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);
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
