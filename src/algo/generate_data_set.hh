#pragma once
/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <numeric>
# include <random>
# include <thread>
# include <vector>


/* local includes  */
# include "../data_set.hh"
# include "../global.hh"
# include "../gazebo.hh"
# include "../log.hh"
# include "../math_tools.hh"
# include "quadcopter.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class Generator {
  
  
public:
    
  Generator(std::vector<std::shared_ptr<flight_controller_t>> quads,
	    std::shared_ptr<simulator_t> sim_interface_); 
  /*Move it from here*/
  Quadcopter<Gazebo>::Reward
  action_evaluator(lt::triangle<double> old_dist,
		   lt::triangle<double> new_dist);

  void move_action(std::string label,
		   Quadcopter<Gazebo>::Action action);  

  
  void phase_one(bool random_leader_action); /* Find a better name for this function */
  
  void run(); //bool random_leader_action

  
private:
    
  int count_;
  int episode_;
  DataSet data_set_;
  std::uniform_real_distribution<> distribution_;
  std::uniform_int_distribution<> distribution_int_;
  std::vector<lt::triangle<double>> f3_side_;
  std::random_device random_dev;  
  std::mt19937 generator_;
  Math_tools mtools_;
  std::vector<std::shared_ptr<flight_controller_t>> quads_;  
  std::shared_ptr<simulator_t> sim_interface_;
  float speed_;  
  
};











