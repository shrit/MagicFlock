#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <future>
#include <thread>
#include <vector>

/* local includes  */
#include <MagicFlock/algorithms/ann_action_predictor.hpp>
#include <MagicFlock/algorithms/ann_state_predictor.hpp>
#include <MagicFlock/controller/quadrotor.hpp>
#include <MagicFlock/controller/swarm_device.hpp>
#include <MagicFlock/metrics/max_distance.hpp>
#include <MagicFlock/metrics/min_distance.hpp>
#include <MagicFlock/util/arma_helper.hpp>
#include <MagicFlock/util/logger.hpp>
#include <MagicFlock/util/scheduler.hpp>
#include <MagicFlock/util/time.hpp>
#include <MagicFlock/util/time_steps.hpp>
#include <MagicFlock/util/real_time_samples.hpp>

template<class QuadrotorType>
class Iterative_learning
{
public:
  Iterative_learning(std::vector<QuadrotorType>& quadrotors,
                     std::shared_ptr<spdlog::logger> logger);

  void execute_trajectory(QuadrotorType& quad);
  void run(std::function<void(void)> func);
  void start_predicting_model();
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
  double elapsed_time_, passed_time_, episode_time_;
  Scheduler sched;
  SwarmDevice<QuadrotorType> swarm_;
  ignition::math::Vector3d dest_ ;
  std::vector<QuadrotorType>& quadrotors_;
  std::shared_ptr<spdlog::logger> logger_; 
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;
  std::mt19937 generator_;
  ArmaHelper arma_;
  RTSamples  predict_sampler_;
};

#include "iterative_learning_impl.hpp"
