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
#include <MagicFlock/simulator/gazebo/ResetModel.pb.h>
#include <MagicFlock/util/logger.hpp>
#include <MagicFlock/util/time.hpp>

template<class QuadrotorType>
class Gazebo
{

public:
  using SubPtr = gazebo::transport::SubscriberPtr;
  using PubPtr = gazebo::transport::PublisherPtr;
  using NodePtr = gazebo::transport::NodePtr;

  /**
   * Gazebo class constructor. It is used to initiallize quadrotors
   * and client node in order to connect to gazebo server.
   */
  Gazebo(std::vector<QuadrotorType>& quadrotors);

  /**
   * This class is necessary to connect this software to gazebo server
   */  
  void Setup(int argc, char* argv[]);

  /**
   * This function subscribe to the position and time topic that are published
   * by gazebo.
   */
  void subsPosTimeTopic();

  /**
   * This function subscribe to the RSSI received by each quadrotor.
   * The function will loop into all the quadrotors and subscribe each
   * quadrotors to its own topic.
   */
  void subRxTopic();

  /**
   * This function subscribe to the Lidar values received by each quadrotor.
   * The function will loop into all the quadrotors and subscribe each
   * quadrotors to its own topic.
   */
  void subLaserTopic();

 /**
   * This function publish (advertise permenantly) the mode reset topic
   * that we need to use in order to reset the quadrotors to their initial 
   * positions.
   */
  void pubModelReset();

  /**
   * This function is used to send reset message to gazebo simulator
   * it uses the reset model plugin which is located in gazebo/ dir
   */
  void ResetModels();

  /**
   * This function is used to parse the received time msg from 
   * gazebo simulator.
   */
  void TimeMsg(ConstWorldStatisticsPtr& msg);
  
  /**
   * The objective of this function is to parse te position message revcieved from
   * gazebo. We are not looking only for the positions of each quadrotors only
   * but we are interested in recovering the 3 antenna positions, the transmitter
   * antenna and the two receiver antennas.
   * Position messages received from gazebo topics
   */  
  void PosMsg(ConstPosesStampedPtr& posesStamped);

  /**
   * This function is used to start the gazebo simulator,
   * It uses the simulation script that is locate in the script/ dir
   */
  void start_simulation(std::string path_2_simulation_script,
                        int n,
                        std::string quadrotor_type);

  /**
   * This function stop the gazebo simnulator, if it has already invoked
   */
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

#include "gazebo_impl.hpp"
