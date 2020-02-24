#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <thread>
#include <vector>

/* local includes  */
#include <ILMR/action_generator.hh>
#include <ILMR/ann_enhanced_predictor.hh>
#include <ILMR/evaluate_model.hh>
#include <ILMR/global.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>
#include <ILMR/time_steps.hh>
#include <ILMR/timer.hh>

namespace lt = local_types;

template<class flight_controller_t, class simulator_t>
class Iterative_learning
{
public:
  Iterative_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
                     const std::vector<Quadrotor<simulator_t>>& quadrotors,
                     std::shared_ptr<simulator_t> gzs,
                     std::shared_ptr<spdlog::logger> logger);

  void generate_trajectory_using_model();
  void run();

  Iterative_learning(Iterative_learning const&) = delete;
  Iterative_learning(Iterative_learning&&) = default;

private:
  int episode_;
  EvaluateModel<simulator_t> evaluate_model_;
  std::vector<double> flight_errors_;
  int max_episode_;
  std::shared_ptr<simulator_t> sim_interface_;
  std::vector<double> step_errors_;
  TimeSteps time_steps_;
  Timer timer_;
  SwarmDevice<flight_controller_t> swarm_;
  bool start_episode_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;

  std::shared_ptr<spdlog::logger> logger_;
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_2_;
};

#include "iterative_learning.hxx"
