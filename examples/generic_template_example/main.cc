/**
 * @file main.cc
 * @brief A working template that provide a generic example to show the user how
 * to start a simulation
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
#include <chrono>
#include <string>
#include <vector>

/*  local defined include */
#include "example.hh"

/* ILMR library include  */
#include <ILMR/gazebo.hh>
#include <ILMR/logger.hh>
#include <ILMR/px4_device.hh>
#include <ILMR/quadrotor.hh>

int
main(int argc, char* argv[])
{
  /*Start using the logging system*/
  auto logger = ILMR::logger::init();

  /* The same logger is used in the example and in the library. The user have to
   * create this function in order to enable logging in the library side*/
  ILMR::logger::create_library_logger(logger);

  /*  We have to create a Gazebo simulator object, in order to subscribe to
   * position information published by gazebo for each quadrotor */
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc, argv);

  /* Subscribe to position information published by gazebo*/
  gz->subscriber();

  /* Publish the reset plugin, this plugin will allow to reset the quadrotors at
   * the end of each episode*/
  gz->publish_model_reset();

  /* Waiting for 10 seconds, Just to finish subscribing and publishing
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));

  /*  Create a vector of quadrotors, each one has an id + a name  */
  /*  Try to see if it is possible or efficient to merge quadrotors +
      device controller */
  std::vector<Quadrotor<Gazebo>> quadrotors;
  quadrotors.emplace_back("leader");
  quadrotors.emplace_back("follower_1");
  quadrotors.emplace_back("follower_2");
  quadrotors.emplace_back("leader_2");

  /*  Add neighbors list for each quadrotor */
  /* Users can decide as the want the neighbors list for each quadrotors as the
   * want, or according to initial positions and configurations for each
   * quadrotors*/
  quadrotors.at(0).add_nearest_neighbor_id(1);
  quadrotors.at(0).add_nearest_neighbor_id(2);
  quadrotors.at(0).add_nearest_neighbor_id(3);

  quadrotors.at(1).add_nearest_neighbor_id(0);
  quadrotors.at(1).add_nearest_neighbor_id(2);
  quadrotors.at(1).add_nearest_neighbor_id(3);

  quadrotors.at(2).add_nearest_neighbor_id(0);
  quadrotors.at(2).add_nearest_neighbor_id(1);
  quadrotors.at(2).add_nearest_neighbor_id(3);

  quadrotors.at(3).add_nearest_neighbor_id(0);
  quadrotors.at(3).add_nearest_neighbor_id(1);
  quadrotors.at(3).add_nearest_neighbor_id(2);

  /*  Start simulation episodes to fly quadrotors... */
  Example<Px4Device> example(quadrotors, gz, logger);
  example.run();
}
