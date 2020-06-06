#pragma once

/*  Standard C++ includes  */
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

/* ILMR includes  */
#include <ILMR/gazebo.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>
#include <ILMR/time.hh>
#include <ILMR/time_steps.hh>

template<class QuadrotorType>
class Generator
{

public:
  Generator(std::vector<QuadrotorType>& quadrotors,
            std::shared_ptr<spdlog::logger> logger);

  void go_to_destination();
  void run(std::function<void(void)> func);

  Generator(Generator const&) = delete;
  Generator(Generator&&) = default;

private:
  int episode_;
  int max_episode_;
  bool start_episode_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType>& quadrotors_;
  TimeSteps time_steps_;
  Timer timer_;
  std::shared_ptr<spdlog::logger> logger_;
};

#include "generate_data_set.hxx"
