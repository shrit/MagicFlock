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
#include <MagicFlock/util/arma_helper.hpp>
#include <MagicFlock/util/logger.hpp>
#include <MagicFlock/controller/quadrotor.hpp>

using namespace ILMR;

template<class QuadrotorType>
class AnnPredictor
{
public:
  AnnPredictor(QuadrotorType& quad);

  arma::mat state_features_matrix();
  arma::mat action_features_matrix();

  arma::mat mig_vel_state_features_matrix();
  arma::mat cohsep_vel_state_features_matrix();

  arma::mat mig_vel_action_features_matrix();
  arma::mat cohsep_vel_action_features_matrix();

  template<typename State>
  arma::mat create_state_matrix(State state, arma::uword matrix_size);

  double compute_real_loss(const arma::mat& labels, arma::uword index);
  double compute_absolute_loss(const arma::mat& labels, arma::uword index);
  double compute_square_loss(const arma::mat& labels, arma::uword index);

protected:
  typename QuadrotorType::Action action_;
  arma::colvec loss_vector_;
  QuadrotorType& quad_;
  ArmaHelper arm_;

};

#include "ann_predictor_impl.hpp"
