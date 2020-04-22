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
#include <ILMR/gazebo.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>

/*  CLI11 library headers */
#include <CLI/CLI.hpp>

/*
 *  Main file: Start generate dataset
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

  /*  Gazebo simulator */
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc, argv);
  gz->subscribe_position_topic();
  gz->publishe_model_reset();

  /* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));

  /*  Create a vector of quadrotors, each one has an id + a name  */
  using QuadrotorType = Quadrotor<Px4Device, GaussianNoise<arma::vec>>;
  std::vector<QuadrotorType> quadrotors;
  quadrotors.emplace_back("leader");     // Alice
  quadrotors.emplace_back("follower_1"); // Charlie
  quadrotors.emplace_back("follower_2"); // Bob
  quadrotors.emplace_back("leader_2_");  // Delta

  /*  Add neighbors list  */
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

  /*  Test the trained model and improve it  */
  Iterative_learning<QuadrotorType> ilearning(quadrotors, logger);
  ilearning.run();
}
