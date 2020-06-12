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

/* C++ includes*/
#include <memory>
#include <mutex>
#include <vector>

/* C includes */
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

/* ILMR include */
#include <ILMR/logger.hh>

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

  Gazebo(std::vector<QuadrotorType>& quadrotors);

  void Setup(int argc, char* argv[]);
  void subsPosTimeTopic();
  void subRxTopic();
  void pubModelReset();
  void ResetModels();

  void TimeMsg(ConstWorldStatisticsPtr& msg);
  void PosMsg(ConstPosesStampedPtr& posesStamped);

  void start_simulation(std::string path_2_simulation_script,
                        int n,
                        std::string quadrotor_type);
  void stop_simulation();

  Gazebo(Gazebo const&) = delete;
  Gazebo(Gazebo&&) = default;

private:
  mutable std::mutex _position_msg_mutex{};
  std::vector<SubPtr> subs_;
  PubPtr pubs_;
  NodePtr node_;
  int sitl_process = 0;
  std::vector<QuadrotorType>& quadrotors_;
};

#include "gazebo.hxx"
