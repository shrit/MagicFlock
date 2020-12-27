#pragma once

/*  Standard C++ includes  */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <thread>
#include <tuple>
#include <vector>

/*  MLPack includes */
#include <mlpack/core.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
/* local includes */
#include "ann_predictor.hpp"
#include <MagicFlock/controller/quadrotor.hpp>
#include <MagicFlock/util/logger.hpp>

using namespace ILMR;

template<class QuadrotorType>
class AnnActionPredictor : public virtual AnnPredictor<QuadrotorType>
{
public:
  AnnActionPredictor(QuadrotorType& quad);

  typename QuadrotorType::Action best_predicted_action(
    std::string full_path_to_model,
    std::string model_name);

  typename QuadrotorType::Action best_predicted_mig_action(
    std::string full_path_to_model,
    std::string model_name);

  typename QuadrotorType::Action best_predicted_cohsep_action(
    std::string full_path_to_model,
    std::string model_name);

  AnnActionPredictor(AnnActionPredictor const&) = delete;
  AnnActionPredictor(AnnActionPredictor&&) = default;
};

#include "ann_action_predictor_impl.hpp"
