#pragma once

/*  Standard C++ includes  */
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

/* ILMR includes  */
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/controller/swarm_device.hpp>
#include <IL4MRC/simulator/gazebo.hpp>
#include <IL4MRC/util/logger.hpp>
#include <IL4MRC/util/time.hpp>
#include <IL4MRC/util/time_steps.hpp>

template<class QuadrotorType>
class Flock
{

public:
  Flock(std::vector<QuadrotorType>& quadrotors,
        std::shared_ptr<spdlog::logger> logger);

  void go_to_destination();
  void run(std::function<void(void)> func);

  Flock(Flock const&) = delete;
  Flock(Flock&&) = default;

private:
  int episode_;
  int max_episode_;
  bool start_episode_;
  double passed_time_, elapsed_time_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType>& quadrotors_;
  TimeSteps time_steps_;
  Timer timer_;
  ignition::math::Vector3d dest_;
  std::shared_ptr<spdlog::logger> logger_;
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;
  std::mt19937 generator_;
};

#include "flocking_test_impl.hpp"
