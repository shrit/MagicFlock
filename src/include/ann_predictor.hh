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
#include "logger.hh"
#include "one_hot_encoding.hh"
#include "quadrotor.hh"

namespace lt = local_types;
using namespace ILMR;

template<class QuadrotorType>
class AnnPredictor
{
public:
  AnnPredictor(typename std::vector<QuadrotorType>::iterator quad);

  arma::mat create_features_matrix();

  template<typename State>
  arma::mat create_state_matrix(State state, arma::uword matrix_size);

  double compute_real_loss(const arma::mat& labels, arma::uword index);
  double compute_absolute_loss(const arma::mat& labels, arma::uword index);
  double compute_square_loss(const arma::mat& labels, arma::uword index);

protected:
  Actions action_;
  arma::colvec loss_vector_;
  typename std::vector<QuadrotorType>::iterator quad_;

private:
  OneHotEncoding one_hot_;
};

#include "ann_predictor.hxx"
