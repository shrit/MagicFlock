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

/* local includes  */
#include "ann_error_predictor.hh"
#include "ann_state_predictor.hh"
#include "argmin.hh"
#include "logger.hh"
#include "quadrotor.hh"

template<class QuadrotorType>
class AnnEnhancedPredictor
  : public AnnStatePredictor<QuadrotorType>
  , public AnnErrorPredictor<QuadrotorType>
{
public:
  AnnEnhancedPredictor(
    std::string full_path_to_state_model,
    std::string state_model_name,
    std::string full_path_to_error_model,
    std::string error_model_name,
    const QuadrotorType& quad);

  arma::mat predict();

  double real_time_loss();

  typename QuadrotorType::Action best_predicted_action();

  arma::vec best_predicted_state();
  arma::Col<arma::uword> all_predicted_actions() const;

  AnnEnhancedPredictor(AnnEnhancedPredictor const&) = delete;
  AnnEnhancedPredictor(AnnEnhancedPredictor&&) = default;

private:
  typename QuadrotorType::Action best_action_follower_;
  arma::uword best_action_index_;
  double real_time_loss_;
  arma::vec loss_vector_;
  arma::Col<arma::uword> all_predicted_actions_;
  arma::mat enhanced_prediction_matrix_;
};

#include "ann_enhanced_predictor.hxx"
