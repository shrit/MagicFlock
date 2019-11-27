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
# include "action.hh"
# include "../global.hh"
# include "../log.hh"
# include "../math_tools.hh"
# include "quadrotor.hh"
# include "reward.hh"
# include "state.hh"
# include "../settings.hh"
# include "../swarm_device.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class Generator {

public:

  Generator(std::vector<std::shared_ptr<flight_controller_t>> quads,
	    const std::vector<Quadrotor<simulator_t>>& quadrotors,
	    std::shared_ptr<simulator_t> sim_interface);

  void generate_trajectory(bool random_leader_action);

  void run(const Settings& settings);

  Generator(Generator const&) = delete;

  Generator(Generator &&) = default;

private:
  
  int count_;
  int episode_;
  int max_episode_;
  Math_tools mtools_;
  std::shared_ptr<simulator_t> sim_interface_;
  bool stop_episode_;  
  SwarmDevice<flight_controller_t> swarm_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;
  
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;
};

# include "generate_data_set.hxx"
