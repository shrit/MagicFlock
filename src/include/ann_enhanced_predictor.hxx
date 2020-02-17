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
  , real_time_loss_(0)
{
  // Nothing to do here.
}

template<class simulator_t>
AnnEnhancedPredictor<simulator_t>::predict()
{
  AnnStatePredictor<simulator_t>::predict();
  arma::mat state_matrix = AnnStatePredictor<simulator_t>::prediction_matrix();

  AnnErrorPredictor<simulator_t>::predict();
  arma::mat error_matrix = AnnErrorPredictor<simulator_t>::prediction_matrix();

  arma::mat enhanced_prediction_matrix = state_matrix + error_matrix;

}
