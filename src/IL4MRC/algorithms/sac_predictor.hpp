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
#include <mlpack/methods/reinforcement_learning/sac.hpp>
#include <mlpack/methods/ann/loss_functions/empty_loss.hpp>
#include <mlpack/methods/ann/init_rules/gaussian_init.hpp>
#include <mlpack/methods/reinforcement_learning/environment/env_type.hpp>
#include <mlpack/methods/reinforcement_learning/training_config.hpp>

/* local includes */
#include <IL4MRC/controller/quadrotor.hpp>
#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename ReplayType = mlpack::rl::RandomReplay<EnvironmentType>,
         typename QuadrotorType
         >
class SACPredictor
{
public:
  SACPredictor(const QuadrotorType& quad);
  void SacNetwork();
  void train(std::function<void(void)> execute_action,
             std::function<double(void)> evaluate_reward,
             std::function<double(void)> examine_environment);

  typename QuadrotorType::Action best_predicted_action();

  SACPredictor(SACPredictor const&) = delete;
  SACPredictor(SACPredictor&&) = default;

private:
  double episodeReturn_;
  mlpack::rl::TrainingConfig config_;
  std::vector<double> returnList_;
};

#include "sac_predictor_impl.hpp"
