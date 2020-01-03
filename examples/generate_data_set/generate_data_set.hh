#pragma once

/*  Standard C++ includes  */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <thread>
#include <vector>

/* ILMR includes  */
#include <ILMR/action.hh>
#include <ILMR/global.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>
#include <ILMR/time_steps.hh>

namespace lt = local_types;

template<class flight_controller_t, class simulator_t>
class Generator
{

public:
  Generator(std::vector<std::shared_ptr<flight_controller_t>> quads,
            const std::vector<Quadrotor<simulator_t>>& quadrotors,
            std::shared_ptr<simulator_t> sim_interface,
            std::shared_ptr<spdlog::logger> logger);

  void generate_trajectory(bool random_leader_action);
  void run();

  Generator(Generator const&) = delete;
  Generator(Generator&&) = default;

private:
  int episode_;
  int max_episode_;
  bool start_episode_;
  std::shared_ptr<simulator_t> sim_interface_;
  SwarmDevice<flight_controller_t> swarm_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;
  TimeSteps time_steps_;
  std::shared_ptr<spdlog::logger> logger_;
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;
};

#include "generate_data_set.hxx"
