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

/* local includes  */
#include "ann_predictor.hh"
#include "global.hh"
#include "logger.hh"
#include "math_tools.hh"
#include "quadrotor.hh"

namespace lt = local_types;
using namespace ILMR;

template<class simulator_t>
class AnnErrorPredictor public: AnnPredictor<simulator_t>
{
public:
  AnnErrorPredictor(
    std::string full_path_to_model,
    std::string model_name,
    typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  std::tuple<arma::mat, arma::uword, Actions::Action> predict(
    arma::mat& features);

  double real_time_loss(
    std::tuple<arma::mat, arma::uword, Actions::Action> matrix_best_action);
  double real_time_loss() const;

  std::vector<double> loss_vector() const;
  Actions::Action get_predicted_action();

  AnnErrorPredictor(AnnErrorPredictor const&) = delete;
  AnnErrorPredictor(AnnErrorPredictor&&) = default;

private:
  std::string model_path_;
  std::string model_name_;
  double real_time_loss_;
  std::vector<double> loss_vector_;
};

#include "ann_error_predictor.hh"
