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
# include "../log.hh"
# include "../math_tools.hh"
# include "quadcopter.hh"
# include "../settings.hh"
# include "../swarm_device.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class Generator {

public:

  Generator(std::vector<std::shared_ptr<flight_controller_t>> quads,
	    std::shared_ptr<simulator_t> sim_interface);

  void generate_trajectory(bool random_leader_action);

  void run(const Settings& settings);

  Generator(Generator const&) = delete;

  Generator(Generator &&) = default;

private:
  
  std::vector<Quadcopter::Action> action_follower_;
  int count_;
  DataSet data_set_;
  int episode_;
  std::vector<lt::triangle<double>> f3_side_;
  int max_episode_;
  Math_tools mtools_;
  std::shared_ptr<simulator_t> sim_interface_;
  std::vector<Quadcopter::State<simulator_t>> states_;
  Quadcopter::Action saved_leader_action_;
  Quadcopter::Action saved_follower_action_;
  bool stop_episode_;
  SwarmDevice<flight_controller_t> swarm_;
};

# include "generate_data_set.hxx"
