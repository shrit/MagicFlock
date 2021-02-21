#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <future>
#include <thread>
#include <vector>

/* local includes  */
#include <MagicFlock/algorithms/ann_action_predictor.hpp>
#include <MagicFlock/controller/quadrotor.hpp>
#include <MagicFlock/controller/swarm_device.hpp>
#include <MagicFlock/metrics/max_distance.hpp>
#include <MagicFlock/metrics/min_distance.hpp>
#include <MagicFlock/util/logger.hpp>
#include <MagicFlock/util/time.hpp>

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
  MaxDistance<QuadrotorType> max_distance_;
  MinDistance<QuadrotorType> min_distance_;
  int max_episode_;
  Timer timer_;
  double elapsed_time_, passed_time_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType>& quadrotors_;
  std::shared_ptr<spdlog::logger> logger_; 
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;
  std::mt19937 generator_;
};

#include "iterative_learning_impl.hpp"
