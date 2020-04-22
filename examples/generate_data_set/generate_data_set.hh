#pragma once

/*  Standard C++ includes  */
#include <chrono>
#include <thread>
#include <vector>

/* ILMR includes  */
#include <ILMR/action_generator.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>
#include <ILMR/time_steps.hh>
#include <ILMR/timer.hh>

template<class QuadrotorType>
class Generator
{

public:
  Generator(const std::vector<QuadrotorType>& quadrotors,
            std::shared_ptr<spdlog::logger> logger);

  void generate_trajectory();
  void run();

  Generator(Generator const&) = delete;
  Generator(Generator&&) = default;

private:
  int episode_;
  int max_episode_;
  bool start_episode_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType> quadrotors_;
  TimeSteps time_steps_;
  Timer timer_;
  std::shared_ptr<spdlog::logger> logger_;
  typename std::vector<QuadrotorType>::iterator leader_;
  typename std::vector<QuadrotorType>::iterator follower_1_;
  typename std::vector<QuadrotorType>::iterator follower_2_;
  typename std::vector<QuadrotorType>::iterator leader_2_;
};

#include "generate_data_set.hxx"
