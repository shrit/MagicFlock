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
#include "global.hh"
#include "logger.hh"
#include "math_tools.hh"
#include "quadrotor.hh"

template<class simulator_t>
class AnnEnhancedPredictor
  : public AnnStatePredictor<simulator_t>
  , public AnnErrorPredictor<simulator_t>
{
public:
  AnnEnhancedPredictor(
    std::string full_path_to_state_model,
    std::string state_model_name,
    std::string full_path_to_error_model,
    std::string error_model_name,
    typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  arma::mat predict();

  double compute_loss();
  double real_time_loss();

  Actions::Action best_predicted_action();

  AnnEnhancedPredictor(AnnEnhancedPredictor const&) = delete;
  AnnEnhancedPredictor(AnnEnhancedPredictor&&) = default;

private:
  Actions::Action best_action_follower_;
  arma::uword best_action_index_;  
  double real_time_loss_;
  arma::vec loss_vector_;
  arma::mat enhanced_prediction_matrix_;
};

#include "ann_enhanced_predictor.hxx"
