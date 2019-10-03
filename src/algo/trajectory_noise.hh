#pragma once

/*  Standard C++ includes  */
# include <random>
# include <vector>

/* local includes  */
# include "../data_set.hh"
# include "../global.hh"
# include "../log.hh"
# include "../settings.hh"
# include "../swarm_device.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class TrajectoryNoise {

public:

  TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
		  std::shared_ptr<simulator_t> sim_interface);
  
  void run();

  TrajectoryNoise(TrajectoryNoise const&) = delete;
  
  TrajectoryNoise(TrajectoryNoise &&) = default;

private:

  std::shared_ptr<simulator_t> sim_interface_;
  SwarmDevice<flight_controller_t> swarm_;
};

# include "trajectory_noise.hxx"
