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
class SignalNoise
{

public:
  SignalNoise(std::vector<QuadrotorType>& quadrotors,
              std::shared_ptr<spdlog::logger> logger);
  void calculate_RSSI_mean_variance();
  void run(std::function<void(void)> func);

  SignalNoise(SignalNoise const&) = delete;
  SignalNoise(SignalNoise&&) = default;

private:
  int episode_;
  int max_episode_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType>& quadrotors_;
  std::vector<arma::colvec> mean_all_quad_;
  std::vector<arma::colvec> stddev_all_quad_;
  std::vector<arma::colvec> var_all_quad_;
  std::shared_ptr<spdlog::logger> logger_;
};

#include "signal_noise_impl.hpp"
