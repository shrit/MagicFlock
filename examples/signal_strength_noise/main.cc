/**
 * @file main.cc
 * @brief Mesure the noise of signal strenght
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
#include "signal_noise.hpp"

/* ILMR library include  */
#include <IL4MRC/actions/continuous_actions.hpp>
#include <IL4MRC/controller/px4_device.hpp>
#include <IL4MRC/metrics/exponential_moving_average.hpp>
#include <IL4MRC/simulator/gazebo.hpp>

/*  CLI11 library headers */
#include <IL4MRC/third_party/CLI11.hpp>

/*
 *  Main file: Measure the noise caused by the signal strenght
 */
int
main(int argc, char* argv[])
{
  /*Start using the new loggin system*/
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  CLI::App app{
    "This example shows how to use the library to measure the noise of the RSSI"
    "This example requires to specify the number of quadrotors to simulate."
  };

  std::size_t num_of_quads = 3;
  bool verbose = false;
  app.add_option("-n, --number_of_quadrotors",
                 num_of_quads,
                 " Number of quadrotor to create inside the simulator.");
  app.add_flag("-v, --verbose", verbose, " Make the output more verbose");

  //CLI11_PARSE(app, argc, argv);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ArgumentMismatch& e) {
    std::cout << e.what() << std::endl;
    return app.exit(e);
  }

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

  /*  Generate a dataset  */
  SignalNoise<QuadrotorType> signal(quadrotors, logger);
  // Callback to be executed at the end of the episode
  signal.run([&]() { gz.ResetModels(); });
  return 0;
}
