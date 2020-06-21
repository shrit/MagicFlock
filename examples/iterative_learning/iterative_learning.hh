#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <thread>
#include <vector>

/* local includes  */
#include <ILMR/ann_enhanced_predictor.hh>
#include <ILMR/evaluate_model.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>
#include <ILMR/time_steps.hh>
#include <ILMR/time.hh>

template<class QuadrotorType>
class Iterative_learning
{
public:
  Iterative_learning(const std::vector<QuadrotorType>& quadrotors,
                     std::shared_ptr<spdlog::logger> logger);

  void generate_trajectory_using_model();
  void run(std::function<void(void)> func);

  Iterative_learning(Iterative_learning const&) = delete;
  Iterative_learning(Iterative_learning&&) = default;

private:
  int episode_;
  std::vector<double> flight_errors_;
  int max_episode_;
  std::vector<double> step_errors_;
  TimeSteps time_steps_;
  Timer timer_;
  SwarmDevice<QuadrotorType> swarm_;
  bool start_episode_;
  std::vector<QuadrotorType> quadrotors_;
  std::shared_ptr<spdlog::logger> logger_; 
};

#include "iterative_learning.hxx"
