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
std::vector<double>
AnnStatePredictor<simulator_t>::estimate_action_from_distance(arma::mat& matrix)
{
  std::vector<double> sum_of_distances;
  double d1, d2, d3;
  double height_diff;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    d1 = std::fabs(this->quad_->all_states().at(0).distances_3D().at(0) -
                   matrix(i, 0));
    d2 = std::fabs(this->quad_->all_states().at(0).distances_3D().at(1) -
                   matrix(i, 1));
    d3 = std::fabs(this->quad_->all_states().at(0).distances_3D().at(2) -
                   matrix(i, 2));
    height_diff = std::fabs(
      this->quad_->all_states().at(0).height_difference() - matrix(i, 3));
    sum_of_distances.push_back(d1 + d3 + height_diff);
  }
  return sum_of_distances;
}

template<class simulator_t>
int
AnnStatePredictor<simulator_t>::index_of_best_action(arma::mat& matrix)
{
  std::vector<double> distances = estimate_action_from_distance(matrix);
  std::reverse(distances.begin(), distances.end());
  logger::logger_->info("Sum of distances: {}", distances);
  int value =
    std::min_element(distances.begin(), distances.end()) - distances.begin();
  return value;
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

  /* Transpose to the original format */
  features = features.t();
  labels_ = labels_.t();

  logger::logger_->info("Size of State features matrix: {}", arma::size(features));
  logger::logger_->info("State data matrix: {}", features);
  logger::logger_->info("State prediction matrix: {}", labels_);

  arma::uword value = index_of_best_action(labels_);
  logger::logger_->info("Index of best action: {}", value);

  label_index_of_best_estimation_ = (labels_.n_rows - 1) - value;

  /*  Get the follower action now !! and store it directly */
  best_action_follower_ = this->action_.int_to_action(value);
  return labels_;
}

template<class simulator_t>
arma::vec
AnnStatePredictor<simulator_t>::best_predicted_state()
{
  arma::vec current_predicted_state;
  current_predicted_state << labels_(label_index_of_best_estimation_, 0)
                          << labels_(label_index_of_best_estimation_, 1)
                          << labels_(label_index_of_best_estimation_, 2)
                          << labels_(label_index_of_best_estimation_, 3);

  return current_predicted_state;
}

template<class simulator_t>
Actions::Action
AnnStatePredictor<simulator_t>::best_predicted_action()
{
  /* Predict the next state using the above data */
  predict();
  this->quad_->current_predicted_state(best_predicted_state());  
  return best_action_follower_;
}

template<class simulator_t>
double
AnnStatePredictor<simulator_t>::compute_loss()
{
  loss_vector_.clear();
  loss_vector_ << labels_(label_index_of_best_estimation_, 0) -
                            this->quad_->current_state().distances_3D().at(0)
               << labels_(label_index_of_best_estimation_, 1) -
                            this->quad_->current_state().distances_3D().at(1)
               << labels_(label_index_of_best_estimation_, 2) -
                            this->quad_->current_state().distances_3D().at(2)
               << labels_(label_index_of_best_estimation_, 3) -
                            this->quad_->current_state().height_difference();


  this->quad_->current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class simulator_t>
double
AnnStatePredictor<simulator_t>::real_time_loss()
{
  real_time_loss_ = compute_loss();
  return real_time_loss_;
}

template<class simulator_t>
arma::vec
AnnStatePredictor<simulator_t>::loss_vector() const
{
  return loss_vector_;
}
