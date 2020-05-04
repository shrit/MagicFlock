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

  std::size_t num_of_quads = 5;

  using QuadrotorType = Quadrotor<Px4Device, GaussianNoise<arma::vec>>;

  /*  Create a vector of quadrotors, each one has an id + a label  */
  std::vector<std::shared_ptr<QuadrotorType>> quadrotors;
  for (std::size_t i = 0; i < num_of_quads; ++i) {
    quadrotors.emplace_back(std::make_shared<QuadrotorType>(i, "iris_" + std::to_string(i), ""));
  }

  /*  Gazebo simulator */
  std::shared_ptr<Gazebo<QuadrotorType>> gz =
    std::make_shared<Gazebo<QuadrotorType>>(quadrotors);
  gz->start_simulation(
    "/meta/lemon/script/gazebo_sitl_multiple_run.sh", num_of_quads, "iris");
  gz->Setup(argc, argv);
  gz->subsPosTimeTopic();
  gz->subRxTopic();
  gz->pubModelReset();

  /* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  logger->info("Waiting for 60 seconds until gazebo start and spawining finish");
  std::this_thread::sleep_for(std::chrono::seconds(60));
  ILMR::logger::logger_->info(
    "Communciation established with simulator"); 

  /*  Generate a dataset  */
  Generator<QuadrotorType> generator(quadrotors, logger);
  generator.run();
  return 0;
}
