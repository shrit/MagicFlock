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
# include "quadrotor.hh"

namespace lt = local_types;

template<class simulator_t>
using states_vec = std::vector<Quadcopter::State<simulator_t>>;

class Predictor
{
public:
      
  Predictor(string name);
  
  template<class simulator_t>
  arma::mat
  create_absolute_features_matrix(const states_vec<simulator_t>& states,
				  Quadrotor::Action follower_action);

  template<class simulator_t>
  arma::mat
  create_estimated_features_matrix(const states_vec<simulator_t>& states,
				   Quadrotor::Action follower_action);

  std::vector<double>
  estimate_action_from_distance(arma::mat& matrix);
  
  int index_of_best_action_classification(arma::mat& matrix);
  int index_of_best_action_regression(arma::mat& matrix);

  std::tuple<arma::mat, arma::uword, Quadrotor::Action> predict(arma::mat& features);

  template<class simulator_t>
  double real_time_loss(const states_vec<simulator_t>& states,
			std::tuple<arma::mat, arma::uword, Quadrotor::Action> matrix_best_action);
  
  Predictor(Predictor const&) = delete;
  Predictor(Predictor &&) = default;
private:
  
  bool classification_;
  Math_tools mtools_;
  bool regression_;
  lt::triangle<double> original_dist_;
  double height_diff_;
  Quadrotor robot_;  
};

# include "predictor.hxx"
