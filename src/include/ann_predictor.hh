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
#include <ensmallen_bits/adam/adam_update.hpp>
#include <mlpack/core.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>

/* local includes  */
#include "global.hh"
#include "logger.hh"
#include "math_tools.hh"
#include "quadrotor.hh"

namespace lt = local_types;
using namespace ILMR;

template<class simulator_t>
class AnnPredictor
{
public:
  AnnPredictor(std::string full_path_to_model,
               std::string model_name,
               typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  arma::mat create_features_matrix();
  arma::mat create_error_feature_vector();

  std::vector<double> best_predicted_state(
    std::tuple<arma::mat, arma::uword, Actions::Action>
      predicted_matrix_best_action);

  std::vector<double> estimate_action_from_distance(arma::mat& matrix);

  int index_of_best_action(arma::mat& matrix);

  std::tuple<arma::mat, arma::uword, Actions::Action> predict(
    arma::mat& features);

  double real_time_loss(
    std::tuple<arma::mat, arma::uword, Actions::Action> matrix_best_action);
  double real_time_loss() const;

  std::vector<double> loss_vector() const;
  Actions::Action get_predicted_action();

  AnnPredictor(AnnPredictor const&) = delete;
  AnnPredictor(AnnPredictor&&) = default;

private:
  std::string model_path_;
  std::string model_name_;
  Actions action_;
  Math_tools mtools_;
  double real_time_loss_;
  std::vector<double> loss_vector_;
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;
};

#include "ann_predictor.hxx"
