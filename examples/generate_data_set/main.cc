/**
 * @file main.cc
 * @brief generate data set using the library
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>

/*  local defined include */
#include "generate_data_set.hh"

/* ILMR library include  */
#include <ILMR/gaussian_noise.hh>
#include <ILMR/gazebo.hh>
#include <ILMR/logger.hh>
#include <ILMR/px4_device.hh>
#include <ILMR/quadrotor.hh>

/*
 *  Main file: Start generating dataset
 */

int
main(int argc, char* argv[])
{
  /*Start using the new loggin system*/
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);
  spdlog::set_level(spdlog::level::debug);

  /*  Gazebo simulator */
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc, argv);

  gz->subscribe_position_topic();
  gz->publishe_model_reset();
 
  /* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));

  /*  Create a vector of quadrotors, each one has an id + a label  */
  using QuadrotorType = Quadrotor<Px4Device, GaussianNoise<arma::vec>>;
  std::vector<QuadrotorType> quadrotors;
  quadrotors.emplace_back("leader", gz);
  quadrotors.emplace_back("follower_1", gz);
  quadrotors.emplace_back("follower_2", gz);
  quadrotors.emplace_back("leader_2", gz);
  /*  Add neighbors list -> Variadic template by label*/
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

  /*  Generate a dataset  */
  Generator<QuadrotorType> generator(quadrotors, logger);
  generator.run();
}
