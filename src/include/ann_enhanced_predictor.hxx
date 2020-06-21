#pragma once

template<class QuadrotorType>
AnnEnhancedPredictor<QuadrotorType>::AnnEnhancedPredictor(
  std::string full_path_to_state_model,
  std::string state_model_name,
  std::string full_path_to_error_model,
  std::string error_model_name,
  typename std::vector<QuadrotorType>::iterator quad)
  : AnnStatePredictor<QuadrotorType>(full_path_to_state_model,
                                   state_model_name,
                                   quad)
  , AnnErrorPredictor<QuadrotorType>(full_path_to_error_model,
                                   error_model_name,
                                   quad)
  , AnnPredictor<QuadrotorType>(quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnEnhancedPredictor<QuadrotorType>::predict()
{
  arma::mat predicted_state = AnnStatePredictor<QuadrotorType>::predict();
  arma::mat predicted_error = AnnErrorPredictor<QuadrotorType>::predict();

  enhanced_prediction_matrix_.clear();
  enhanced_prediction_matrix_ = predicted_state + predicted_error;

  arma::mat original_state_matrix = this->create_state_matrix(
    this->quad_->all_states().at(0), predicted_error.n_cols);

  Argmin<arma::mat, arma::uword> argmin(
    original_state_matrix, enhanced_prediction_matrix_, 1);

  best_action_index_ = argmin.min_index();

  best_action_follower_ = this->action_.int_to_action(best_action_index_);

  arma::Col<arma::uword> temp;
  temp << best_action_index_;
  all_predicted_actions_.insert_rows(all_predicted_actions_.n_rows, temp);

  return enhanced_prediction_matrix_;
}

template<class QuadrotorType>
arma::vec
AnnEnhancedPredictor<QuadrotorType>::best_predicted_state()
{
  return enhanced_prediction_matrix_.col(best_action_index_);
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnEnhancedPredictor<QuadrotorType>::best_predicted_action()
{
  predict();
  this->quad_->current_predicted_enhanced_state().Data() =
    best_predicted_state();
  return best_action_follower_;
}

template<class QuadrotorType>
arma::Col<arma::uword>
AnnEnhancedPredictor<QuadrotorType>::all_predicted_actions() const
{
  return all_predicted_actions_;
}

template<class QuadrotorType>
double
AnnEnhancedPredictor<QuadrotorType>::real_time_loss()
{
  real_time_loss_ =
    this->compute_real_loss(enhanced_prediction_matrix_, best_action_index_);
  return real_time_loss_;
}
