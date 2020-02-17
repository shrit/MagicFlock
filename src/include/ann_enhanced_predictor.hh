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
#include "ann_predictor.hh"
#include "ann_state_predictor.hh"
#include "ann_error_predictor.hh"
#include "global.hh"
#include "logger.hh"
#include "math_tools.hh"
#include "quadrotor.hh"


template<class simulator_t>
class AnnEnhancedePredictor : public AnnStatePredictor<simulator_t>, AnnErrorPredictor<simulator_t>
{
public:
  AnnEnhancedePredictor(
    std::string full_path_to_state_model,
    std::string state_model_name,
    std::string full_path_to_error_model,
    std::string error_model_name,    
    typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  arma::vec best_predicted_state();

  std::vector<double> estimate_action_from_distance(arma::mat& matrix);

  int index_of_best_action(arma::mat& matrix);

  Actions::Action predict(arma::mat& features);

  void compute_loss();  
  double real_time_loss() const;

  arma::vec loss_vector() const;
  Actions::Action best_predicted_action();

  AnnEnhancedePredictor(AnnEnhancedePredictor const&) = delete;
  AnnEnhancedePredictor(AnnEnhancedePredictor&&) = default;

private:
 
  double real_time_loss_;
  arma::vec loss_vector_;
  arma::uword label_index_of_best_estimation_;
};
