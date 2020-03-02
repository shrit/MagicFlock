#pragma once

template<class simulator_t>
AnnStatePredictor<simulator_t>::AnnStatePredictor(
  std::string full_path_to_model,
  std::string model_name,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : AnnPredictor<simulator_t>(quad)
  , real_time_loss_(0)
  , model_path_(full_path_to_model)
  , model_name_(model_name)
{
  // Nothing to do here.
}

template<class simulator_t>
arma::mat
AnnStatePredictor<simulator_t>::predict()
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path_, model_name_, regression_model, true);
  arma::mat features = this->create_features_matrix();

  /*  We need to predict the action for the follower using h(S)*/
  /*  Extract state and push it into the model with several actions */
  /*  Take the action index for the highest class
      given back by the model */
  labels_.clear();
  regression_model.Predict(features, labels_);

  logger::logger_->info("Size of State features matrix: {}",
                        arma::size(features));
  logger::logger_->info("State data matrix:\n {}", features.t());
  logger::logger_->info("State prediction matrix:\n {}", labels_.t());

  arma::mat original_state_matrix = this->create_state_matrix(
    this->quad_->all_states().at(0), labels_.n_cols);

  Argmin<arma::mat, arma::uword> argmin(original_state_matrix, labels_, 1);

  best_action_index_ = argmin.min_index();
  logger::logger_->info("Index of best action: {}", best_action_index_);
 
  /*  Get the follower action now !! and store it directly */
  best_action_follower_ = this->action_.int_to_action(best_action_index_);
  return labels_;
}

template<class simulator_t>
arma::vec
AnnStatePredictor<simulator_t>::best_predicted_state()
{
  return labels_.col(best_action_index_);
}

template<class simulator_t>
Actions::Action
AnnStatePredictor<simulator_t>::best_predicted_action()
{
  /* Predict the next state using the above data */
  predict();
  this->quad_->current_predicted_state.Data() = best_predicted_state();
  return best_action_follower_;
}

template<class simulator_t>
double
AnnStatePredictor<simulator_t>::compute_real_loss()
{
  loss_vector_.clear();
  loss_vector_ = this->quad_->current_state().Data() -
                 labels_.col(best_action_index_);

  this->quad_->current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class simulator_t>
double
AnnStatePredictor<simulator_t>::compute_absolute_loss()
{
  loss_vector_.clear();
  loss_vector_ = arma::abs(this->quad_->current_state().Data() -
                           labels_.col(best_action_index_));

  this->quad_->current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class simulator_t>
double
AnnStatePredictor<simulator_t>::compute_square_loss()
{
  loss_vector_.clear();
  mlpack::ann::MeanSquaredError<arma::rowvec, arma::rowvec> mse;
  double error = mse.Forward(this->quad_->current_state().Data(),
                             labels_.col(best_action_index_));

  loss_vector_ = arma::square(this->quad_->current_state().Data() -
                              labels_.col(best_action_index_));

  this->quad_->current_loss(loss_vector_);
  return error;
}

template<class simulator_t>
double
AnnStatePredictor<simulator_t>::real_time_loss()
{
  real_time_loss_ = compute_real_loss();
  return real_time_loss_;
}

template<class simulator_t>
arma::vec
AnnStatePredictor<simulator_t>::loss_vector() const
{
  return loss_vector_;
}
