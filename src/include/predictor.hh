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
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>

/*  Armadillo includes  */
# include <armadillo>

/* local includes  */
# include "global.hh"
# include "log.hh"
# include "math_tools.hh"
# include "quadrotor.hh"

namespace lt = local_types;

template<class simulator_t>
using quadrotors_vec = std::vector<Quadrotor<simulator_t>>;

template<class simulator_t>
class Predictor
{
public:
      
  Predictor(std::string name,
	    std::string full_path_to_model,
	    std::string model_name,
	    typename std::vector<Quadrotor<simulator_t>>::iterator quad);
  
  arma::mat
  create_absolute_features_matrix();
				  
  arma::mat
  create_estimated_features_matrix();

  std::vector<double>
  estimate_action_from_distance(arma::mat& matrix);
  
  int index_of_best_action_classification(arma::mat& matrix);
  int index_of_best_action_regression(arma::mat& matrix);

  std::tuple<arma::mat, arma::uword, Actions::Action> predict(arma::mat& features);

  double real_time_loss(std::tuple<arma::mat, arma::uword, Actions::Action> matrix_best_action);
  double real_time_loss() const;

  Actions::Action get_predicted_action();
  
  Predictor(Predictor const&) = delete;
  Predictor(Predictor &&) = default;
  
private:
  std::string model_path_;
  std::string model_name_;
  Actions action_;
  bool classification_;
  Math_tools mtools_;
  bool regression_;
  std::vector<double> original_dist_;
  double height_diff_;
  double real_time_loss_;
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;

};

# include "predictor.hxx"
