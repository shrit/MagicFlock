#pragma once

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <numeric>
# include <random>
# include <thread>
# include <tuple>
# include <vector>

/*  MLPack includes */
#include <mlpack/core.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>

/*  Armadillo includes  */
# include <armadillo>

/* local includes  */
# include "../global.hh"
# include "../log.hh"
# include "../math_tools.hh"
# include "quadcopter.hh"

namespace lt = local_types;

class Predictor
{
public:
  Predictor();
  
  arma::mat
  create_absolute_features_matrix(std::vector<Quadcopter::Action> actions);
  
  arma::mat
  create_estimated_features_matrix(std::vector<Quadcopter::Action> actions);

  std::vector<double>
  estimate_action_from_distance(arma::mat matrix);
  
  int index_of_best_action_classification(arma::mat matrix);
  int index_of_best_action_regression(arma::mat matrix);

  std::tuple<arma::mat, arma::uword> predict(arma::mat features);
  
  double real_time_loss(std::tuple<arma::mat, arma::uword> matrix_best_action);
  
  Predictor(Predictor const&) = delete;
  Predictor(Predictor &&) = default;
private:
  
  bool classification_;
  Math_tools mtools_;
  bool regression_;
  Quadcopter::Action saved_leader_action_;
  std::shared_ptr<simulator_t> sim_interface_;
  lt::triangle<double> original_dist_;
  double height_diff_;
  Quadcopter robot_;  
};

# include "predictor.hxx"
