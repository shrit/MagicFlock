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
#include <ILMR/config_ini.hh>
#include <ILMR/gazebo.hh>
#include <ILMR/gaussian_noise.hh>
#include <ILMR/logger.hh>
#include <ILMR/px4_device.hh>
#include <ILMR/quadrotor.hh>

/*
 *  Main file: Start generate dataset
 */
using port_type = std::uint16_t;

int
main(int argc, char* argv[])
{
  /*  Init configs */
  Configs configs("/meta/lemon/quad.ini");

  /*Start using the new loggin system*/
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);
  spdlog::set_level(spdlog::level::debug);
  std::vector<port_type> ports = configs.quads_ports();

  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */
  std::vector<std::shared_ptr<Px4Device>> iris_x;

  for (auto& it : ports) {
    iris_x.push_back(std::make_shared<Px4Device>("udp", it));
    logger::logger_->info("Add an iris Quadrotor!");
  }

  logger::logger_->info("Ports number: {}", ports);
  /*  Gazebo simulator */
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc, argv, configs);

  gz->subscriber(configs.positions());

  /* Verify the numbers to subscribe to the good signal strength */
  gz->subscriber(configs.rssi_1_2());
  gz->subscriber(configs.rssi_1_3());
  gz->subscriber(configs.rssi_2_3());

  gz->publisher(configs.reset_1());
  gz->publisher(configs.reset_2());
  gz->publisher(configs.reset_3());
  gz->publisher(configs.reset_4());

/* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));

  /*  Create a vector of quadrotors, each one has an id + a name  */
  /*  Try to see if it is possible or efficient to merge quadrotors +
      device controller */

  using QuadrotorType = Quadrotor<Gazebo, GaussianNoise<arma::vec>>;
  std::vector<QuadrotorType> quadrotors;
  quadrotors.emplace_back(0, "leader", gz);
  quadrotors.emplace_back(1, "follower_1", gz);
  quadrotors.emplace_back(2, "follower_2", gz);
  quadrotors.emplace_back(3, "leader_2", gz); 
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
  
/*  Generate a dataset  */
  Generator<Px4Device, QuadrotorType> generator(iris_x, quadrotors, logger);
  generator.run();
}
