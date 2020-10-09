#pragma once

/*  Standard C++ includes  */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <numeric>
#include <random>
#include <thread>
#include <tuple>
#include <vector>

/*  mlpack includes */
#include <mlpack/core.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/init_rules/gaussian_init.hpp>
#include <mlpack/methods/ann/loss_functions/empty_loss.hpp>
#include <mlpack/methods/reinforcement_learning/environment/env_type.hpp>
#include <mlpack/methods/reinforcement_learning/sac.hpp>
#include <mlpack/methods/reinforcement_learning/training_config.hpp>

/* local includes */
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename QuadrotorType,
         typename ReplayType = mlpack::rl::RandomReplay<EnvironmentType>>
class SACPredictor
{
public:
  SACPredictor(QuadrotorType& quad);
  void SacNetwork();
  void train(
    size_t& consecutiveEpisodes,
    const size_t numSteps,
    std::function<void()> execute_action,
    std::function<double()> evaluate_reward,
    std::function<bool()> examine_environment);
  void test();

  SACPredictor(SACPredictor const&) = delete;
  SACPredictor(SACPredictor&&) = default;

private:
  double episodeReturn_;
  mlpack::rl::TrainingConfig config_;
  std::vector<double> returnList_;
  mlpack::rl::RandomReplay<EnvironmentType> replayMethod_;
  QuadrotorType& quadrotor_;
  std::unique_ptr<
    mlpack::rl::SAC<EnvironmentType, NetworkType, UpdaterType, PolicyType>>
    agent_;
   mlpack::ann::FFN<mlpack::ann::EmptyLoss<>,
                   mlpack::ann::GaussianInitialization> policyNetwork_, qNetwork_;
};

#include "sac_predictor_impl.hpp"
