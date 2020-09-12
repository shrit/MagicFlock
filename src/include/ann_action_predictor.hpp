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
#include "ann_predictor.hh"
#include "logger.hh"
#include "quadrotor.hh"

using namespace ILMR;

template<class QuadrotorType>
class AnnActionPredictor : public virtual AnnPredictor<QuadrotorType>
{
public:
  AnnActionPredictor(
    std::string full_path_to_model,
    std::string model_name,
    const QuadrotorType& quad);

  arma::mat predict();

  typename QuadrotorType::Action best_predicted_action();

  AnnActionPredictor(AnnActionPredictor const&) = delete;
  AnnActionPredictor(AnnActionPredictor&&) = default;

private:
  arma::mat labels_;
  std::string model_path_;
  std::string model_name_;
  double real_time_loss_;
  arma::vec loss_vector_;
  arma::uword best_action_index_;
  typename QuadrotorType::Action best_action_;
  arma::Col<arma::uword> all_predicted_actions_;
};

#include "ann_action_predictor_impl.hpp"
