/**
 * @file main.cc
 * @brief Test the model and improve it using the library
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

/*  local  defined include */
#include "iterative_learning.hh"

/* ILMR library include  */
#include <ILMR/continuous_actions.hh>
#include <ILMR/exponential_moving_average.hh>
#include <ILMR/gazebo.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/px4_device.hh>

/*  CLI11 library headers */
#include <ILMR/CLI11.hpp>

/*
 *  Main file: Start iterative learning
 */
int
main(int argc, char* argv[])
{
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  CLI::App app{
    "This example shows how to use the library to do iterative learning."
    "This example requires two model files to be used to generate trajectory."
    "Users need to generate dataset, then train it using the training example "
    "to aquire these model files."
  };
  std::size_t num_of_quads = 3;
  bool verbose = false;
  app.add_option("-n, --number_of_quadrotors",
                 num_of_quads,
                 " Number of quadrotor to create inside the simulator.");
  app.add_flag("-v, --verbose", verbose, " Make the output more verbose");

  
  app.require_subcommand(1);
  auto model_files =
    app.add_subcommand("model_files",
                       "Add model files, the first one is for follower 1 and "
                       "the second one is for follower 2. etc...");

  std::vector<std::string> model_files_name;
  model_files->add_option("files", model_files_name, "Model files to add");

  model_files->callback([&]() {
    std::cout << "Adding:";
    if (model_files_name.empty()) {
      std::cout << "No files have been added" << std::endl;
      exit(0);
    }
  });

  CLI11_PARSE(app, argc, argv);

 if (verbose) {
    spdlog::set_level(spdlog::level::debug);
  }

  using QuadrotorType =
    Quadrotor<Px4Device, ExpoMovingAverage<arma::colvec>, ContinuousActions>;

  /*  Create a vector of quadrotors, each one has an id + a label  */
  std::vector<QuadrotorType> quadrotors;
  for (std::size_t i = 0; i < num_of_quads; ++i) {
    QuadrotorType quad;
    quadrotors.push_back(quad);
  }

  for (std::size_t i = 0; i < num_of_quads; ++i) {
    quadrotors.at(i).init(
      i, "iris_" + std::to_string(i), "", num_of_quads, quadrotors);
  }

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

  /*  Test the trained model and improve it  */
  Iterative_learning<QuadrotorType> ilearning(quadrotors, logger);
  ilearning.run([&](){ gz.ResetModels(); });
  return 0;
}
