/**
 * @file main.cc
 * @brief Test the model statistically
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
# include <chrono>
# include <cstdlib>
# include <string>
# include <thread>
# include <vector>

/*  local  defined include */
# include "statistic.hh"

/* ILMR library include  */
# include <ILMR/gazebo.hh>
# include <ILMR/config_ini.hh>
# include <ILMR/quadrotor.hh>
# include <ILMR/logger.hh>
# include <ILMR/px4_device.hh>

/*  CLI11 library headers */
# include <CLI/CLI.hpp>

/*
 *  Main file: Start generate dataset
 */
int main(int argc, char* argv[])
{
  /*  Init configs */
  Configs configs("/meta/lemon/quad.ini");

	/*Start using the new loggin system*/
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  CLI::App app{"This app use statistic method in order to maintain the swarm of quadrotor."};

  std::string dataset_filename = "default";
  app.add_option("-f,--file", dataset_filename, "Full Path to the dataset files for quadrotor")
    ->required()
    ->check(CLI::ExistingFile);

  std::vector<lt::port_type> ports = configs.quads_ports();

  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */
  std::vector<std::shared_ptr<Px4Device>> iris_x;

  for (auto& it : ports) {
    iris_x.push_back(std::make_shared<Px4Device>("udp", it));
    logger->info( "Add an iris QCopter!");
  }

  logger->info("Ports number: {}",ports);

  /*  Gazebo simulator */
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc, argv, configs);

  gz->subscriber(configs.positions());

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
  std::vector<Quadrotor<Gazebo>> quadrotors;
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

/*  Test the trained model and improve it  */
  quadrotors.at(3).add_nearest_neighbor_id(0);
  quadrotors.at(3).add_nearest_neighbor_id(1);
  quadrotors.at(3).add_nearest_neighbor_id(2);  

  Statistic<Px4Device, Gazebo> stat(iris_x, quadrotors, gz, logger);
  stat.run();
}
