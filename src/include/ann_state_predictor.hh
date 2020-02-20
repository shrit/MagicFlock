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
#include "global.hh"
#include "logger.hh"
#include "math_tools.hh"
#include "quadrotor.hh"

namespace lt = local_types;
using namespace ILMR;

template<class simulator_t>
class AnnStatePredictor : public virtual AnnPredictor<simulator_t>
{
public:
  AnnStatePredictor(
    std::string full_path_to_model,
    std::string model_name,
    typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  arma::vec best_predicted_state();

  std::vector<double> estimate_action_from_distance(arma::mat& matrix);

  int index_of_best_action(arma::mat& matrix);

  arma::mat predict();

  double compute_loss();
  double real_time_loss();

  arma::vec loss_vector() const;
  Actions::Action best_predicted_action();

  AnnStatePredictor(AnnStatePredictor const&) = delete;
  AnnStatePredictor(AnnStatePredictor&&) = default;

private:
  arma::mat labels_;  
  std::string model_path_;
  std::string model_name_;
  double real_time_loss_;
  arma::vec loss_vector_;
  arma::uword label_index_of_best_estimation_;
  Actions::Action best_action_follower_;
};

#include "ann_state_predictor.hxx"