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
#include "generate_data_set.hpp"

/* ILMR library include  */
#include <MagicFlock/actions/continuous_actions.hpp>
#include <MagicFlock/controller/px4_device.hpp>
#include <MagicFlock/dists/empty_noise.hpp>
#include <MagicFlock/metrics/cumulative_moving_average.hpp>
#include <MagicFlock/metrics/empty_filter.hpp>
#include <MagicFlock/metrics/exponential_moving_average.hpp>
#include <MagicFlock/simulator/gazebo.hpp>

/*  CLI11 library headers */
#include <MagicFlock/third_party/CLI11.hpp>

/*
 *  Main file: Start generating dataset
 */
int
main(int argc, char* argv[])
{
  /*Start using the new loggin system*/
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  CLI::App app{
    "This example shows how to use the library to generate dataset. "
    "This example requires to specify the number of quadrotors to simulate. "
    "Generated dataset are saved in a dataset folder which contains dataset "
    "generated in each run."
  };

  std::size_t num_of_quads = 3;
  std::size_t num_of_external_radio_src = 0;

  bool verbose = false;
  app.add_option("-n, --number_of_quadrotors",
                 num_of_quads,
                 " Number of quadrotor to create inside the simulator.");
  app.add_option(
    "-a, --number_of_radio",
    num_of_external_radio_src,
    " Number of external radio source inside the simulator."
    "If you added external antenna source specify the number here.");
  app.add_flag("-v, --verbose", verbose, " Make the output more verbose");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ArgumentMismatch& e) {
    std::cout << e.what() << std::endl;
    return app.exit(e);
  }

  if (verbose) {
    spdlog::set_level(spdlog::level::debug);
  }

  using QuadrotorType = Quadrotor<Px4Device,
                                  EmptyFilter<arma::colvec>,
                                  EmptyNoise<arma::colvec>,
                                  FullWiFi,
                                  DiscretActions>;

  /*  Create a vector of quadrotors, each one has an id + a label  */
  std::vector<QuadrotorType> quadrotors;
  for (std::size_t i = 0; i < num_of_quads; ++i) {
    QuadrotorType quad;
    quadrotors.push_back(quad);
  }
  std::vector<ignition::math::Vector3d> fix_antennas(0);

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

  /*  Generate a dataset  */
  Generator<QuadrotorType> generator(quadrotors, logger);
  // Callback to be executed at the end of the episode
  generator.run([&]() { gz.ResetModels(); });
  return 0;
}
