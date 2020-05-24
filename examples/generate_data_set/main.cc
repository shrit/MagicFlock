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
#include <ILMR/continuous_actions.hh>
#include <ILMR/gaussian_noise.hh>
#include <ILMR/gazebo.hh>
#include <ILMR/logger.hh>
#include <ILMR/px4_device.hh>
#include <ILMR/quadrotor.hh>

/*  CLI11 library headers */
#include <CLI/CLI.hpp>

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

  CLI::App app{
    "This example shows how to use the library to generate dataset. "
    "This example requires to specify the number of quadrotors to simulate. "
    "Generated dataset are saved in a dataset folder which contains dataset "
    "generated in each run."
  };

  std::size_t num_of_quads = 3;

  app.add_option("-n, --number_of_quadrotors",
                 num_of_quads,
                 " Number of quadrotor to create inside the simulator.");
  CLI11_PARSE(app, argc, argv);

  using QuadrotorType =
    Quadrotor<Px4Device, GaussianNoise<arma::vec>, ContinuousActions>;

  /*  Create a vector of quadrotors, each one has an id + a label  */
  std::vector<QuadrotorType> quadrotors;
  for (std::size_t i = 0; i < num_of_quads; ++i) {
    QuadrotorType quad;
    quadrotors.push_back(quad);
  }

  for (std::size_t i = 0; i < num_of_quads; ++i) {
    quadrotors.at(i).init(i, "iris_" + std::to_string(i), "");
  }

  for (std::size_t i = 0; i < num_of_quads; ++i) {
    logger->info(quadrotors.at(i).port_number());
  }

  /* Give an access to each quadrotor to all its neighbors when generating
   * dataset */
  // for (auto it : quadrotors) {
  //   it->make_reference_2_swarm(quadrotors);
  // }

  /*  Gazebo simulator */
  Gazebo<QuadrotorType> gz(quadrotors);
  gz.start_simulation(
    "/meta/lemon/script/gazebo_sitl_multiple_run.sh", num_of_quads, "iris");

  logger->info(
    "Waiting for 30 seconds until gazebo start and spawining finish");
  std::this_thread::sleep_for(std::chrono::seconds(30));
  gz.Setup(argc, argv);
  gz.subsPosTimeTopic();
  gz.subRxTopic();
  gz.pubModelReset();

  /* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));

  ILMR::logger::logger_->info("Communciation established with simulator");

  for (std::size_t i = 0; i < num_of_quads; ++i) {
    quadrotors.at(i).start_controller();
  }

  /*  Generate a dataset  */
  Generator<QuadrotorType> generator(quadrotors, logger);
  generator.run();
  return 0;
}
