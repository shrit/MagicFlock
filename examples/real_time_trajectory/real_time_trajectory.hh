#pragma once

/*  Standard C++ includes  */
#include <vector>

/* Local includes  */
#include <ILMR/action.hh>
#include <ILMR/data_set.hh>
#include <ILMR/global.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>
#include <ILMR/time_steps.hh>

namespace lt = local_types;

template<class flight_controller_t, class simulator_t>
class RTTrajectory
{

public:
  RTTrajectory(std::vector<std::shared_ptr<flight_controller_t>> quads,
               const std::vector<Quadrotor<simulator_t>>& quadrotors,
               std::shared_ptr<simulator_t> sim_interface,
               std::shared_ptr<spdlog::logger> logger);
  void run();

  void start_trajectory(bool stop_down_action);

  RTTrajectory(RTTrajectory const&) = delete;
  RTTrajectory(RTTrajectory&&) = default;

private:
  int count_;
  DataSet<simulator_t> data_set_;
  int episode_;
  int max_episode_;
  Math_tools mtools_;
  TimeSteps time_steps_;
  Actions action_;
  std::shared_ptr<simulator_t> sim_interface_;
  bool start_episode_;
  SwarmDevice<flight_controller_t> swarm_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;
  std::shared_ptr<spdlog::logger> logger_;
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;
};

#include "real_time_trajectory.hxx"
