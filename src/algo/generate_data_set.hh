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

  void move_action(std::string label,
		   typename Quadcopter<simulator_t>::Action action);  

  
  void phase_one(bool random_leader_action); /* Find a better name for this function */
      
  typename Quadcopter<simulator_t>::Action
  randomize_action();
  
  void run();

  Generator(Generator const&) = delete;  
  
  Generator(Generator &&) = default;
  
  
private:
  /*  Intilizate speed with configs settings */
  std::vector<typename Quadcopter<simulator_t>::Action> action_follower_ ;
  int count_;
  DataSet data_set_;
  std::vector<double> drift_f3_;
  std::uniform_real_distribution<> distribution_;
  std::uniform_int_distribution<> distribution_int_;
  int episode_;
  std::vector<lt::triangle<double>> f3_side_;
  std::random_device random_dev;  
  std::mt19937 generator_;
  int max_episode_;  
  Math_tools mtools_;
  std::vector<std::shared_ptr<flight_controller_t>> quads_;  
  std::shared_ptr<simulator_t> sim_interface_;
  float speed_;
  std::vector<typename Quadcopter<simulator_t>::State> states_;
  typename Quadcopter<simulator_t>::Action saved_leader_action_;  
  
};

# include "generate_data_set.hxx"









