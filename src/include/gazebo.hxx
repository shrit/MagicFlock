#pragma once

template<class QuadrotorType>
Gazebo<QuadrotorType>::Gazebo(
  int argc,
  char* argv[],
  std::vector<std::shared_ptr<QuadrotorType>> quadrotors)
  : node_(new gazebo::transport::Node())
  , quadrotors_(quadrotors)
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
  for (auto it : quadrotors_) {
    std::string topic_WR_1 = it->wireless_receiver_1_topic_name();
    subs_.push_back(
      node_->Subscribe(topic_WR_1, &Gazebo<QuadrotorType>::RxMsgN1, this));
    std::string topic_WR_2 = it->wireless_receiver_2_topic_name();
    subs_.push_back(
      node_->Subscribe(topic_WR_2, &Gazebo<QuadrotorType>::RxMsgN2, this));
  }
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::pubModelReset()
{
  for (auto it : quadrotors_) {
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
      ILMR::logger::logger->error(
        "NO Connection from the subscriber to reset the model");
    }
  }
}

/*
 * There is a small problem in rssi from rssi_from_neighbors
 */

/*  Parsing the RSSI send by Gazebo */
template<class QuadrotorType>
void
Gazebo<QuadrotorType>::RxMsgN1(const ConstWirelessNodesPtr& _msg)
{
  std::lock_guard<std::mutex> lock(_rx_mutex);
  this->_RxNodesMsg = _msg;
  int numRxNodes = _RxNodesMsg->node_size();

  for (int i = 0; i < numRxNodes; ++i) {
    gazebo::msgs::WirelessNode RxNode = _RxNodesMsg->node(i);
    quadrotors_.at(i)->rssi_from_neighbors().name = RxNode.essid();
    quadrotors_.at(i)->rssi_from_neighbors().antenna_1 = RxNode.signal_level();
  }
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::RxMsgN2(const ConstWirelessNodesPtr& _msg)
{
  std::lock_guard<std::mutex> lock(_rx_mutex);
  this->_RxNodesMsg = _msg;
  int numRxNodes = _RxNodesMsg->node_size();

  for (int i = 0; i < numRxNodes; ++i) {
    for (int j = 0; j < numRxNodes; ++j) {
      gazebo::msgs::WirelessNode RxNode = _RxNodesMsg->node(j);
      quadrotors_.at(i)->rssi_from_neighbors().name = RxNode.essid();
      quadrotors_.at(i)->rssi_from_neighbors().antenna_2.at(j) =
        RxNode.signal_level();
    }
  }
}

/*  Position messages received from gazebo topics */
template<class QuadrotorType>
void
Gazebo<QuadrotorType>::PosMsg(ConstPosesStampedPtr& posesStamped)
{
  for (int i = 0; i < posesStamped->pose_size(); ++i) {
    const ::gazebo::msgs::Pose& pose = posesStamped->pose(i);
    std::string name = pose.name();
    for (std::size_t j = 0; j < quadrotors_.size(); ++j) {
      if (name == std::string(quadrotors_.at(j)->names()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->position() = ::gazebo::msgs::ConvertIgn(position);

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
        quadrotors_.at(j)->orientation() =
          ::gazebo::msgs::ConvertIgn(orientation);

      } else if (name = std::string(quadrotors_.at(j)->wt_name()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->wt_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);

      } else if (name == std::string(quadrotors_.at(j)->wr_1_name()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->wr_1_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);

      } else if (name == std::string(quadrotors_.at(j)->wr_2_name()) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->wr_2_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);
      }
    }
  }
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::TimeMsg(ConstWorldStatisticsPtr& msg)
{
  Time t(msg->sim_time().sec(), msg->sim_time().nsec());
  Time rt(msg->real_time().sec(), msg->real_time().nsec());
  Time::setTime(t, t.seconds() / rt.seconds());
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::start_simulation(int n)
{
  int p = fork();
  if (p == 0) { // Child
    char* const bash = (char*)"/bin/bash";
    char* const script = (char*)"../script/gazebo_sitl_multiple_run.sh";
    char num[16];
    strcpy(num, std::to_string(n).c_str());

    char* const argv[] = { bash, script, "-n", num, "-m", "iris", NULL };

    execv(bash, argv);

    exit(0);
    return;
  }

  sitl_process = p;
}

template<class QuadrotorType>
void
Gazebo<QuadrotorType>::stop_simulation()
{
  if (sitl_process == 0)
    return;

  kill(sitl_process, SIGINT);
  waitpid(sitl_process, NULL, 0);
}
