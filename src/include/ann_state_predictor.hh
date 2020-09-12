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
#include "argmin.hh"
#include "logger.hh"
#include "quadrotor.hh"

using namespace ILMR;

template<class QuadrotorType>
class AnnStatePredictor : public virtual AnnPredictor<QuadrotorType>
{
public:
  AnnStatePredictor(
    std::string full_path_to_model,
    std::string model_name,
    QuadrotorType& quad);

  arma::vec best_predicted_state();

  arma::Col<arma::uword> all_predicted_actions() const;

  arma::mat predict();
  
  double real_time_loss();

  arma::vec loss_vector() const;
  typename QuadrotorType::Action best_predicted_action();

  AnnStatePredictor(AnnStatePredictor const&) = delete;
  AnnStatePredictor(AnnStatePredictor&&) = default;

private:
  arma::mat labels_;
  std::string model_path_;
  std::string model_name_;
  double real_time_loss_;
  arma::vec loss_vector_;
  arma::uword best_action_index_;
  typename QuadrotorType::Action best_action_follower_;
  arma::Col<arma::uword> all_predicted_actions_;
};

#include "ann_state_predictor_impl.hpp"
