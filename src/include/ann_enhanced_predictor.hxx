#pragma once

template<class simulator_t>
AnnEnhancedPredictor<simulator_t>::AnnEnhancedPredictor(
  std::string full_path_to_state_model,
  std::string state_model_name,
  std::string full_path_to_error_model,
  std::string error_model_name,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : AnnStatePredictor<simulator_t>(full_path_to_state_model,
                                   state_model_name,
                                   quad)
  , AnnErrorPredictor<simulator_t>(full_path_to_error_model,
                                   error_model_name,
                                   quad)
  , AnnPredictor<simulator_t>(quad)
{
  // Nothing to do here.
}

template<class simulator_t>
arma::mat
AnnEnhancedPredictor<simulator_t>::predict()
{
  arma::mat predicted_state = AnnStatePredictor<simulator_t>::predict();
  arma::mat predicted_error = AnnErrorPredictor<simulator_t>::predict();

  enhanced_prediction_matrix_.clear();
  enhanced_prediction_matrix_ = predicted_state + predicted_error;
  arma::mat original_state_matrix =
    this->create_state_matrix(predicted_error.n_cols);
  Argmin<arma::mat, arma::uword> argmin(
    original_state_matrix, enhanced_prediction_matrix_, 1);
  arma::uword best_action_index = argmin.min_index();
  index_of_best_estimation_ = best_action_index;
  best_action_follower_ = this->action_.int_to_action(best_action_index);
  return enhanced_prediction_matrix_;
}

template<class simulator_t>
Actions::Action
AnnEnhancedPredictor<simulator_t>::best_predicted_action()
{
  predict();
  return best_action_follower_;
}

template<class simulator_t>
double
AnnEnhancedPredictor<simulator_t>::compute_loss()
{
  loss_vector_.clear();

  loss_vector_ << enhanced_prediction_matrix_(index_of_best_estimation_, 0) -
                    this->quad_->current_state().distances_3D().at(0)
               << enhanced_prediction_matrix_(index_of_best_estimation_, 1) -
                    this->quad_->current_state().distances_3D().at(1)
               << enhanced_prediction_matrix_(index_of_best_estimation_, 2) -
                    this->quad_->current_state().distances_3D().at(2)
               << enhanced_prediction_matrix_(index_of_best_estimation_, 3) -
                    this->quad_->current_state().height_difference();

  this->quad_->current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class simulator_t>
double
AnnEnhancedPredictor<simulator_t>::real_time_loss()
{
  real_time_loss_ = compute_loss();
  return real_time_loss_;
}
