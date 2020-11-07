#pragma once

template<class QuadrotorType>
AnnStatePredictor<QuadrotorType>::AnnStatePredictor(
  std::string full_path_to_model,
  std::string model_name,
  QuadrotorType& quad)
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
                   mlpack::ann::GlorotInitialization>
    regression_model;

  mlpack::data::Load(model_path_, model_name_, regression_model, true);
  arma::mat features = this->create_features_matrix();

  if constexpr (std::is_same<typename QuadrotorType::Action,
                             ContinuousActions>::value) {
    labels_.clear();

    logger::logger_->info("Size of State features matrix: {}",
                          arma::size(features));
    logger::logger_->info("State data matrix:\n {}", features.t());
    regression_model.Predict(features, labels_);

    logger::logger_->info("State prediction matrix:\n {}", labels_.t());
    arma::colvec prefect_data = {2,2,2,2,2,2,2,2,2,2,2,2,0,0};
    arma::colvec reduced_prefect_data = {2,2,2,2,0,0};
    this->quad_.all_states().at(0).Data() = prefect_data;
    arma::mat original_state_matrix =
      this->create_state_matrix(this->quad_.all_states().at(0), labels_.n_cols);
    Argmin<arma::mat, arma::uword> argmin(original_state_matrix, labels_, 1);

    best_action_index_ = argmin.min_index();
    logger::logger_->info("Index of best action: {}", best_action_index_);

    /*  Get the follower action now !! and store it directly */
    best_action_follower_ = this->action_.int_to_action(best_action_index_);

  } else if constexpr (std::is_same<typename QuadrotorType::Action,
                                    DiscretActions>::value) {

    /**
     * In this function we predict all the possible next states using
     * regression model, each next state is related to a specific action.
     * Finally, we do an argmin between the original state and all of the
     * next states observed here.
     * The action which correspond to the best state is returned by argmin
     * and returned finally by the best action function
     * The objective of this function is to predict only next states
     * This in only valid in the case of discret actions.
     */
    labels_.clear();
    regression_model.Predict(features, labels_);

    logger::logger_->info("Size of State features matrix: {}",
                          arma::size(features));
    logger::logger_->info("State data matrix:\n {}", features.t());
    logger::logger_->info("State prediction matrix:\n {}", labels_.t());

    arma::mat original_state_matrix =
      this->create_state_matrix(this->quad_.all_states().at(0), labels_.n_cols);

    Argmin<arma::mat, arma::uword> argmin(original_state_matrix, labels_, 1);

    best_action_index_ = argmin.min_index();
    logger::logger_->info("Index of best action: {}", best_action_index_);

    /*  Get the follower action now !! and store it directly */
    best_action_follower_ = this->action_.int_to_action(best_action_index_);

    arma::Col<arma::uword> temp;
    temp << best_action_index_;
    all_predicted_actions_.insert_rows(all_predicted_actions_.n_rows, temp);
  }
  return labels_;
}

template<class QuadrotorType>
arma::vec
AnnStatePredictor<QuadrotorType>::best_predicted_state()
{
  /* Return the best state based on the best action discovered in predict */
  return labels_.col(best_action_index_);
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnStatePredictor<QuadrotorType>::best_predicted_action()
{
  /* Return the best action using the above predict function */
  predict();
  this->quad_.current_predicted_state().Data() = best_predicted_state();
  return best_action_follower_;
}

template<class QuadrotorType>
arma::Col<arma::uword>
AnnStatePredictor<QuadrotorType>::all_predicted_actions() const
{
  /* Return a vector of all best predicted actions in the past */
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
