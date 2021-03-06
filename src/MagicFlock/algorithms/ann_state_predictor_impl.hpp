#pragma once

template<class QuadrotorType>
AnnStatePredictor<QuadrotorType>::AnnStatePredictor(QuadrotorType& quad)
  : AnnPredictor<QuadrotorType>(quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnStatePredictor<QuadrotorType>::shed_angles(arma::mat labels, bool leader)
{
  arma::mat tmp;
  if (leader) {
    tmp.insert_rows(tmp.n_rows, labels.row(0));

  } else {
    tmp.insert_rows(tmp.n_rows, labels.row(0));
    tmp.insert_rows(tmp.n_rows, labels.row(3));
    tmp.insert_rows(tmp.n_rows, labels.row(6));
    tmp.insert_rows(tmp.n_rows, labels.row(9));
    tmp.insert_rows(tmp.n_rows, labels.row(11));
  }
  return tmp;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnStatePredictor<QuadrotorType>::best_predicted_action(std::string model_path,
                                                        std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::GlorotInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->state_features_matrix();

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
  arma::mat labels;

  logger::logger_->info("Size of State features matrix: {}",
                        arma::size(features));
  // logger::logger_->info("State data matrix:\n {}", features.t());
  regression_model.Predict(features, labels);

  // logger::logger_->info("State prediction matrix:\n {}", labels.t());
  arma::colvec prefect_data = { 2, 2, 2, 2, 2 };
  arma::mat distance_labels = shed_angles(labels, false);
  this->quad_.all_states().at(0).Data() = prefect_data;
  arma::mat original_state_matrix = this->create_state_matrix(
    this->quad_.all_states().at(0), distance_labels.n_cols);
  Argmin<arma::mat, arma::uword> argmin(
    original_state_matrix, distance_labels);

  arma::uword best_action_index = argmin.min_index();
  logger::logger_->info("Index of best action: {}", best_action_index);

  /*  Get the follower action now !! and store it directly */
  typename QuadrotorType::Action best_action;
  best_action = this->action_.int_to_action(best_action_index);

  return best_action;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnStatePredictor<QuadrotorType>::best_predicted_cohsep_action(
  std::string model_path,
  std::string model_name)

{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::GlorotInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->cohsep_vel_state_features_matrix();

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
  arma::mat labels;

 /* logger::logger_->info("Size of CohSep State features matrix: {}",
                        arma::size(features));
*/  // logger::logger_->info("State data matrix:\n {}", features.t());
  regression_model.Predict(features, labels);

  // logger::logger_->info("State prediction matrix:\n {}", labels.t());
  arma::colvec perfect_data = { 2, 2, 2, 2, 2 };
  arma::mat distance_labels = shed_angles(labels, false);
  // logger::logger_->info("Distance label prediction matrix:\n {}",
  //                       distance_labels.t());

  arma::mat original_state_matrix =
    this->create_state_matrix(perfect_data, distance_labels.n_cols);
  Argmin<arma::mat, arma::uword> argmin(
    original_state_matrix, distance_labels);

  arma::uword best_action_index = argmin.min_index();
 // logger::logger_->info("Index of best action: {}", best_action_index);

  /*  Get the follower action now !! and store it directly */
  typename QuadrotorType::Action best_action;
  best_action = this->action_.int_to_action(best_action_index);
  return best_action;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnStatePredictor<QuadrotorType>::best_predicted_mig_action(
  std::string model_path,
  std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::GlorotInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->mig_vel_state_features_matrix();

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
  arma::mat labels;

  logger::logger_->info("Size of Mig State features matrix: {}",
                        arma::size(features));
  logger::logger_->info("State MIG feature matrix:\n {}", features.t());
  regression_model.Predict(features, labels);

  logger::logger_->info("State MIG prediction matrix:\n {}", labels.t());
  arma::colvec perfect_data = { 2 };
  arma::mat distance_labels = shed_angles(labels, true);
  logger::logger_->info("Distance label prediction matrix:\n {}",
                        distance_labels.t());
  arma::mat original_state_matrix =
    this->create_state_matrix(perfect_data, distance_labels.n_cols);
  Argmin<arma::mat, arma::uword> argmin(
    original_state_matrix, distance_labels);

  arma::uword best_action_index = argmin.min_index();
  logger::logger_->info("Index of best action: {}", best_action_index);

  /*  Get the follower action now !! and store it directly */
  typename QuadrotorType::Action best_action;
  best_action = this->action_.int_to_action(best_action_index);
  return best_action;
}
