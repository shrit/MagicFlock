#pragma once

/*  Standard C++ includes  */
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

/* ILMR includes  */
#include <MagicFlock/controller/quadrotor.hpp>
#include <MagicFlock/controller/swarm_device.hpp>
#include <MagicFlock/simulator/gazebo.hpp>
#include <MagicFlock/util/logger.hpp>
#include <MagicFlock/util/time.hpp>
#include <MagicFlock/util/time_steps.hpp>

template<class QuadrotorType>
class Generator
{

public:
  Generator(std::vector<QuadrotorType>& quadrotors,
            std::shared_ptr<spdlog::logger> logger);

  void execute_trajectory(QuadrotorType& quad);
  void run(std::function<void(void)> func);

  Generator(Generator const&) = delete;
  Generator(Generator&&) = default;

private:
  int episode_;
  int max_episode_;
  bool start_episode_;
  double passed_time_, elapsed_time_, episode_time_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType>& quadrotors_;
  TimeSteps time_steps_;
  Timer timer_;
  ignition::math::Vector3d dest_;
  std::shared_ptr<spdlog::logger> logger_;
  std::uniform_int_distribution<> distribution_int_, distribution_int_time;
  std::random_device random_dev;
  std::mt19937 generator_;
};

#include "generate_data_set_impl.hpp"
