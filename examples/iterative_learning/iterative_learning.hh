#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <thread>
#include <vector>

/* local includes  */
#include <IL4MRC/algorithms/ann_action_predictor.hpp>
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/controller/swarm_device.hpp>
#include <IL4MRC/metrics/evaluate_model.hpp>
#include <IL4MRC/util/logger.hpp>
#include <IL4MRC/util/time.hpp>
#include <IL4MRC/util/time_steps.hpp>

template<class QuadrotorType>
class Iterative_learning
{
public:
  Iterative_learning(std::vector<QuadrotorType>& quadrotors,
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
  std::vector<QuadrotorType>& quadrotors_;
  std::shared_ptr<spdlog::logger> logger_; 
};

#include "iterative_learning_impl.hpp"
