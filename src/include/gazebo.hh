/*
 * This file contains several functions as a gazebo helpers.
 * It is possible to communicate with gazebo using these functions.
 * This class allows to recover:
 * 1. Positions of quadrotors and on-board sensors.
 * 2. The received signal stregnth by the quadrotrs from other quadrotors.
 * 3. Start the entire simulation and the PX4 autopilot using the provided
 * script in the script directory.
 * 4. Stop the entire simulation.
 * 5. Reset all the quadrotors using the reset plugins.
 * 6. Recover the simulation time and the realtime from gazebo
 * 7. Synchronize the actual software with gazebo.
 *
 *
 * Author: Omar Shrit <shrit@lri.fr>
 *
 */

#pragma once

/* Gazebo and ignition includes*/
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/pose.pb.h>
#include <gazebo/transport/transport.hh>
#include <ignition/math4/ignition/math/Quaternion.hh>
#include <ignition/math6/ignition/math/Vector2.hh>
#include <ignition/math6/ignition/math/Vector3.hh>

/* C++ includes*/
#include <memory>
#include <mutex>
#include <vector>

/* C includes */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

/* Local message includes*/
#include "ResetModel.pb.h"
#include "time.hh"

template<class QuadrotorType>
class Gazebo
{

public:
  using SubPtr = gazebo::transport::SubscriberPtr;
  using PubPtr = gazebo::transport::PublisherPtr;
  using NodePtr = gazebo::transport::NodePtr;

  Gazebo(int argc,
         char* argv[],
         std::vector<std::shared_ptr<QuadrotorType>> quadrotors);

  void subsPosTimeTopic();
  void subRxTopic();
  void pubModelReset();
  void ResetModels();

  void TimeMsg(ConstWorldStatisticsPtr& msg);
  void PosMsg(ConstPosesStampedPtr& posesStamped);
  void RxMsg(ConstWirelessNodesPtr& _msg, int n_antenna);

  void spawn(const std::vector<std::shared_ptr<QuadrotorType>> quadrotors,
             std::string sdf_file,
             std::string rcs_file);

  void start_sitl(int n);
  void stop_sitl();

  Gazebo(Gazebo const&) = delete;
  Gazebo(Gazebo&&) = default;

private:
  std::vector<SubPtr> subs_;
  std::vector<PubPtr> pubs_;
  NodePtr node_;
  int sitl_process = 0;
  mutable std::mutex _rx_mutex{};
  ConstWirelessNodesPtr& _RxNodesMsg;
  std::vector<std::shared_ptr<QuadrotorType>> quadrotors_;
};

#include "gazebo.hxx"
