#pragma once

template<class QuadrotorType>
AnnStatePredictor<QuadrotorType>::AnnStatePredictor(
  std::string full_path_to_model,
  std::string model_name,
  typename std::vector<QuadrotorType>::iterator quad)
  : AnnPredictor<QuadrotorType>(quad)
  , real_time_loss_(0)
  , model_path_(full_path_to_model)
  , model_name_(model_name)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnStatePredictor<QuadrotorType>::predict()
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

  arma::mat original_state_matrix =
    this->create_state_matrix(this->quad_->all_states().at(0), labels_.n_cols);

  Argmin<arma::mat, arma::uword> argmin(original_state_matrix, labels_, 1);

  best_action_index_ = argmin.min_index();
  logger::logger_->info("Index of best action: {}", best_action_index_);

  /*  Get the follower action now !! and store it directly */
  best_action_follower_ = this->action_.int_to_action(best_action_index_);

  arma::Col<arma::uword> temp;
  temp << best_action_index_;
  all_predicted_actions_.insert_rows(all_predicted_actions_.n_rows, temp);

  return labels_;
}

template<class QuadrotorType>
arma::vec
AnnStatePredictor<QuadrotorType>::best_predicted_state()
{
  return labels_.col(best_action_index_);
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnStatePredictor<QuadrotorType>::best_predicted_action()
{
  /* Predict the next state using the above data */
  predict();
  this->quad_->current_predicted_state().Data() = best_predicted_state();
  return best_action_follower_;
}

template<class QuadrotorType>
arma::Col<arma::uword>
AnnStatePredictor<QuadrotorType>::all_predicted_actions() const
{
  return all_predicted_actions_;
}

template<class QuadrotorType>
double
AnnStatePredictor<QuadrotorType>::real_time_loss()
{
  real_time_loss_ = this->compute_real_loss(labels_, best_action_index_);
  return real_time_loss_;
}

template<class QuadrotorType>
arma::vec
AnnStatePredictor<QuadrotorType>::loss_vector() const
{
  return loss_vector_;
}
