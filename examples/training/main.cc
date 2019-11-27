/**
 * @file main.cc
 * @brief generate data set using the library
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

/*  locale defined include */
# include "training.hh"

/*
 *  Main file: Start generate dataset
 */
int main(int argc, char* argv[])
{
  /*  Init configs */
    Configs configs;
    /*  Init logging system */
  Log log;
  log.init();

  std::vector<lt::port_type> ports = configs.quads_ports();
  
  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */
  std::vector<std::shared_ptr<Px4Device>> iris_x;
  
  for (auto& it : ports) {    
    iris_x.push_back(std::make_shared<Px4Device>("udp", it));
    LogInfo() << "Add an iris QCopter! " ;
  }
  
  LogInfo() << "Ports number: "<< ports;      
  
  /*  Gazebo simulator */
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc,argv);
  
  gz->subscriber(configs.positions());
  
  /* Verify the numbers to subscribe to the good signal strength */
  
  gz->subscriber(configs.rssi_1_2());
  gz->subscriber(configs.rssi_1_3());
  gz->subscriber(configs.rssi_2_3());
  
  gz->publisher(configs.reset_1());
  gz->publisher(configs.reset_2());
  gz->publisher(configs.reset_3());
  
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
  
  LogInfo() << "Start training...";
  Train trainer;    
  trainer.load_data_set(settings.dataset());
  trainer.run(settings);
  LogInfo() << "Finished training...";  
}
