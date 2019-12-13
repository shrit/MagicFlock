#pragma once

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <numeric>
# include <thread>
# include <vector>

/* local includes  */

# include <ILMR/statistical_predictor.hh>
# include <ILMR/quadrotor.hh>
# include <ILMR/swarm_device.hh>
# include <ILMR/action.hh>
# include <ILMR/global.hh>
# include <ILMR/log.hh>

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class Statistic
{
public:
  Statistic(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	    const std::vector<Quadrotor<simulator_t>>& quadrotors,
	    std::shared_ptr<simulator_t> gzs);

  void generate_trajectory_using_model(bool random_leader_action,
				       bool stop_down_action);
  
  void run();
  
  Statistic(Statistic const&) = delete;
  Statistic(Statistic &&) = default;

private:
  
  int count_;
  int episode_; 
  int max_episode_ ;
  std::shared_ptr<simulator_t> sim_interface_;
  SwarmDevice<flight_controller_t> swarm_;
  bool stop_episode_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;

  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;
};