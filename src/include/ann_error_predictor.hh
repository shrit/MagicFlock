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
#include "logger.hh"
#include "math_tools.hh"
#include "quadrotor.hh"

namespace lt = local_types;
using namespace ILMR;

template<class QuadrotorType>
class AnnErrorPredictor : public virtual AnnPredictor<QuadrotorType>
{
public:
  AnnErrorPredictor(
    std::string full_path_to_model,
    std::string model_name,
    typename std::vector<QuadrotorType>::iterator quad);

  arma::mat predict();
  arma::colvec predict_specific_action_error(Actions::Action action);

  double real_time_loss() const;

  AnnErrorPredictor(AnnErrorPredictor const&) = delete;
  AnnErrorPredictor(AnnErrorPredictor&&) = default;

private:
  std::string model_path_;
  std::string model_name_;
  double real_time_loss_;
};

#include "ann_error_predictor.hxx"
