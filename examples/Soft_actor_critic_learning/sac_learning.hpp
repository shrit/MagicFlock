#pragma once

/*  Standard C++ includes */
#include <chrono>
#include <thread>
#include <vector>

/* local includes  */
#include <IL4MRC/algorithms/sac_predictor.hpp>
#include <IL4MRC/check/reward.hpp>
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/controller/swarm_device.hpp>
#include <IL4MRC/util/logger.hpp>
#include <IL4MRC/util/time.hpp>
#include <IL4MRC/util/time_steps.hpp>

template<class QuadrotorType>
class SoftActorCritic
{
public:
  SoftActorCritic(std::vector<QuadrotorType>& quadrotors,
                  std::shared_ptr<spdlog::logger> logger);

  void generate_trajectory_using_model();
  void run(std::function<void(void)> func);

  SoftActorCritic(SoftActorCritic const&) = delete;
  SoftActorCritic(SoftActorCritic&&) = default;

private:
  int episode_;
  int max_episode_;
  TimeSteps time_steps_;
  Timer timer_;
  SwarmDevice<QuadrotorType> swarm_;
  std::vector<QuadrotorType>& quadrotors_;
  std::shared_ptr<spdlog::logger> logger_;
  Reward<QuadrotorType> reward_;
  SACPredictor<mlpack::rl::ContinuousActionEnv,
               mlpack::ann::FFN<mlpack::ann::EmptyLoss<>,
                                mlpack::ann::GaussianInitialization>,
               mlpack::ann::FFN<mlpack::ann::EmptyLoss<>,
                                mlpack::ann::GaussianInitialization>,
               ens::AdamUpdate,
               QuadrotorType>
    sac_;
};

#include "sac_learning_impl.hpp"
