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

  using QuadrotorType = Quadrotor<Px4Device, GaussianNoise<arma::vec>>;

  /*  Create a vector of quadrotors, each one has an id + a label  */
  std::vector<std::shared_ptr<QuadrotorType>> quadrotors;
  // quadrotors.emplace_back("leader");
  // quadrotors.emplace_back("follower_1");
  // quadrotors.emplace_back("follower_2");
  // quadrotors.emplace_back("leader_2");
  /*  Add neighbors list -> Variadic template by label*/

  /*  Gazebo simulator */
  std::shared_ptr<Gazebo<QuadrotorType>> gz =
    std::make_shared<Gazebo<QuadrotorType>>(argc, argv, quadrotors);
  gz->start_simulation(
    "/meta/lemon/script/gazebo_sitl_multiple_run.sh", 4, "iris");
  gz->subsPosTimeTopic();
  gz->subRxTopic();
  gz->pubModelReset();

  /* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));
  ILMR::logger::logger_->info(
    "Communciation established with simulator"); 

  /*  Generate a dataset  */
  // Generator<QuadrotorType> generator(quadrotors, logger);
  // generator.run();
}
