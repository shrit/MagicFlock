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
#include "iterative_learning.hpp"

/* ILMR library include  */
#include <IL4MRC/actions/continuous_actions.hpp>
#include <IL4MRC/controller/px4_device.hpp>
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/dists/empty_noise.hpp>
#include <IL4MRC/dists/gaussian_noise.hpp>
#include <IL4MRC/metrics/empty_filter.hpp>
#include <IL4MRC/simulator/gazebo.hpp>
#include <IL4MRC/third_party/CLI11.hpp>

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
  std::size_t num_of_external_radio_src = 0;
  bool verbose = false;
  std::string model_file_name;

  app.add_option("-n, --number_of_quadrotors",
                 num_of_quads,
                 " Number of quadrotor to create inside the simulator.");
  app.add_option(
    "-a, --number_of_radio",
    num_of_external_radio_src,
    " Number of external radio source inside the simulator."
    "If you added external antenna source specify the number here.");
  app.add_option("-m, --model_file", model_file_name, "Model files to add.");
  app.add_flag("-v, --verbose", verbose, " Make the output more verbose");

  CLI11_PARSE(app, argc, argv);

  if (verbose) {
    spdlog::set_level(spdlog::level::debug);
  }

  using QuadrotorType = Quadrotor<Px4Device,
                                  EmptyFilter<arma::colvec>,
                                  EmptyNoise<arma::colvec>,
                                  AntennaDists,
                                  ContinuousActions>;

  /*  Create a vector of quadrotors, each one has an id + a label  */
  std::vector<QuadrotorType> quadrotors;
  for (std::size_t i = 0; i < num_of_quads; ++i) {
    QuadrotorType quad;
    quadrotors.push_back(quad);
  }

  // ignition::math::Vector3d right_antenna{ 10, -90, 45 };
  ignition::math::Vector3d front_antenna{ 100, 0, 45 };
  std::vector<ignition::math::Vector3d> fix_antennas(1);
  // fix_antennas.at(0) = right_antenna;
  fix_antennas.at(0) = front_antenna;

  for (std::size_t i = 0; i < num_of_quads; ++i) {
    quadrotors.at(i).init(i,
                          "iris_" + std::to_string(i),
                          "",
                          num_of_quads,
                          num_of_external_radio_src,
                          fix_antennas,
                          quadrotors);
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
  ilearning.run([&]() { gz.ResetModels(); });
  return 0;
}
