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
#include <ensmallen_bits/adam/adam_update.hpp>
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
  AnnPredictor(typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  arma::mat create_features_matrix();
  arma::mat create_state_matrix();

protected:
  Actions action_;
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;

private:
  Math_tools mtools_;
};

#include "ann_predictor.hxx"
