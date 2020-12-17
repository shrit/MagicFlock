#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <thread>
#include <vector>

/* local includes  */
#include <IL4MRC/algorithms/ann_action_predictor.hpp>
#include <IL4MRC/algorithms/ann_state_predictor.hpp>
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/controller/swarm_device.hpp>
#include <IL4MRC/metrics/max_distance.hpp>
#include <IL4MRC/metrics/min_distance.hpp>
#include <IL4MRC/util/logger.hpp>
#include <IL4MRC/util/time.hpp>
#include <IL4MRC/util/time_steps.hpp>

template<class QuadrotorType>
class Iterative_learning
{
public:
  Iterative_learning(std::vector<QuadrotorType>& quadrotors,
                     std::shared_ptr<spdlog::logger> logger);

  void execute_trajectory(QuadrotorType& quad);
  void run(std::function<void(void)> func);

  Iterative_learning(Iterative_learning const&) = delete;
  Iterative_learning(Iterative_learning&&) = default;

private:
  int episode_;
  std::vector<double> flight_errors_;
  MaxDistance<QuadrotorType> max_distance_;
  MinDistance<QuadrotorType> min_distance_;
  int max_episode_;
  std::vector<double> step_errors_;
  TimeSteps time_steps_;
  Timer timer_;
  bool start_episode_;
  double elapsed_time_, passed_time_;
  SwarmDevice<QuadrotorType> swarm_;
  ignition::math::Vector3d dest_ ;
  std::vector<QuadrotorType>& quadrotors_;
  std::shared_ptr<spdlog::logger> logger_; 
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;
  std::mt19937 generator_;
};

#include "iterative_learning_impl.hpp"
