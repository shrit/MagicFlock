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

/*
 * A logical solution would be creating a gazebo::transport::node inside
 * each quadrotor, and init this node in this costr. Then, a propore
 * implmentation would be possible for each quadrotor to receive data from each
 * rxnode.
 */
template<class QuadrotorType>
void
Gazebo<QuadrotorType>::subRxTopic()
{
  for (auto it : quadrotors_) {
    it->subRxTopic();
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
      msg.set_reset(true);
      it->Publish(msg);
    } else {
      ILMR::logger::logger_->error(
        "NO Connection from the subscriber to reset the model");
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
      if (name == std::string(quadrotors_.at(j)->name())) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->position() = ::gazebo::msgs::ConvertIgn(position);

        const ::gazebo::msgs::Quaternion& orientation = pose.orientation();
        quadrotors_.at(j)->orientation() =
          ::gazebo::msgs::ConvertIgn(orientation);

      } else if (name == std::string(quadrotors_.at(j)->wt_name())) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->wt_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);

      } else if (name == std::string(quadrotors_.at(j)->wr_1_name())) {
        const ::gazebo::msgs::Vector3d& position = pose.position();
        quadrotors_.at(j)->wr_1_antenna_position() =
          ::gazebo::msgs::ConvertIgn(position);

      } else if (name == std::string(quadrotors_.at(j)->wr_2_name())) {
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
Gazebo<QuadrotorType>::start_simulation(std::string path_script,
                                        int n,
                                        std::string type)
{
  int p = fork();
  if (p == 0) { // Child
    char* const bash = (char*)"/bin/bash";
    char* const script = &path_script[0];
    char* const typ = &type[0];
    char num[16];
    strcpy(num, std::to_string(n).c_str());

    char* const argv[] = { bash, script, "-n", num, "-m", typ, NULL };

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
