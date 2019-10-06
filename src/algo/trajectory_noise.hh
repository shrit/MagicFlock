#pragma once

/*  Standard C++ includes  */

/* Local includes  */
//# include "../data_set.hh"
//# include "../global.hh"
//# include "../log.hh"
//# include "../settings.hh"
# include "../swarm_device.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class TrajectoryNoise {

public:

  TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
		  std::shared_ptr<simulator_t> sim_interface);  
  void run();

  void test_trajectory(bool random_action);
  
  TrajectoryNoise(TrajectoryNoise const&) = delete;
  
  TrajectoryNoise(TrajectoryNoise &&) = default;

private:
  int episode_;
  int max_episode_;
  Math_tools mtools_;
  Quadcopter::Action saved_action_;
  std::shared_ptr<simulator_t> sim_interface_;
  bool stop_episode_;
  SwarmDevice<flight_controller_t> swarm_;
};

# include "trajectory_noise.hxx"
