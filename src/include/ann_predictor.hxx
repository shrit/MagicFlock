#pragma once

template<class simulator_t>
AnnPredictor<simulator_t>::AnnPredictor(
  std::string name, /*  Classification or regression */
  std::string full_path_to_model,
  std::string model_name,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : real_time_loss_(0)
  , model_path_(full_path_to_model)
  , model_name_(model_name)
  , quad_(quad)
{
  classification_ = false;
  regression_ = false;
  if (name == "classification") {
    classification_ = true;
  } else if (name == "regression") {
    regression_ = true;
  } else {
    logger::logger_->info("Please entre either classification or regression to "
                          "star the prediction");
  }
}

/* Estimate the features (distances) using propagation model from RSSI */
template<class simulator_t>
arma::mat
AnnPredictor<simulator_t>::create_estimated_features_matrix()
{
  arma::mat features;
  arma::rowvec row;
  std::vector<Actions::Action> actions = action_.all_possible_actions();

  for (int i = 0; i < 7; ++i) {
    /*  State */
    row << quad_->last_state().distances_3D().at(0)
        << quad_->last_state().distances_3D().at(1)
        << quad_->last_state().distances_3D().at(2)
        << quad_->last_state().height_difference()
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(0)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(1)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(2)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(3)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(4)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(5)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(6)
        << quad_->current_state().distances_3D().at(0)
        << quad_->current_state().distances_3D().at(1)
        << quad_->current_state().distances_3D().at(2)
        << quad_->current_state().height_difference()
        /*  Action encoded as 1, and 0, add 7 times to represent 7 actions */
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(0)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(1)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(2)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(3)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(4)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(5)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(6);

    features.insert_rows(0, row);
  }
  /*  We need to transpose the matrix, since mlpack is column major */
  features = features.t();
  return features;
}

template<class simulator_t>
arma::mat
AnnPredictor<simulator_t>::create_absolute_features_matrix()
{
  arma::mat features;
  arma::rowvec row;
  std::vector<Actions::Action> actions = action_.all_possible_actions();

  for (int i = 0; i < 7; ++i) {
    /*  State */
    row << quad_->last_state().distances_3D().at(0)
        << quad_->last_state().distances_3D().at(1)
        << quad_->last_state().distances_3D().at(2)
        << quad_->last_state().height_difference()
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(0)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(1)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(2)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(3)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(4)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(5)
        << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(6)
        << quad_->current_state().distances_3D().at(0)
        << quad_->current_state().distances_3D().at(1)
        << quad_->current_state().distances_3D().at(2)
        << quad_->current_state().height_difference()
        /*  Action encoded as 1, and 0, add 7 times to represent 7 actions */
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(0)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(1)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(2)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(3)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(4)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(5)
        << mtools_.to_one_hot_encoding(actions.at(i), 7).at(6);
    /*  Create a matrix of several rows, each one is added to on the top */
    features.insert_rows(0, row);
  }
  /*  We need to transpose the matrix, since mlpack is column major */
  features = features.t();
  /*  The return features need to be used in the model in order to
      give back the best action with highest score */
  return features;
}

template<class simulator_t>
std::vector<double>
AnnPredictor<simulator_t>::estimate_action_from_distance(arma::mat& matrix)
{
  std::vector<double> sum_of_distances;
  double d1, d2, d3;
  double height_diff;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    d1 =
      std::fabs(quad_->all_states().at(0).distances_3D().at(0) - matrix(i, 0));
    d2 =
      std::fabs(quad_->all_states().at(0).distances_3D().at(1) - matrix(i, 1));
    d3 =
      std::fabs(quad_->all_states().at(0).distances_3D().at(2) - matrix(i, 2));
    height_diff =
      std::fabs(quad_->all_states().at(0).height_difference() - matrix(i, 3));
    sum_of_distances.push_back(d1 + d3 + height_diff);
  }
  return sum_of_distances;
}

template<class simulator_t>
int
AnnPredictor<simulator_t>::index_of_best_action_classification(
  arma::mat& matrix)
{
  int value = 0;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    value = arma::index_max(matrix.col(0));
  }
  return value;
}

template<class simulator_t>
int
AnnPredictor<simulator_t>::index_of_best_action_regression(arma::mat& matrix)
{
  std::vector<double> distances = estimate_action_from_distance(matrix);
  std::reverse(distances.begin(), distances.end());
  logger::logger_->info("Sum of distances: {}", distances);
  int value =
    std::min_element(distances.begin(), distances.end()) - distances.begin();
  return value;
}

template<class simulator_t>
double
AnnPredictor<simulator_t>::real_time_loss(
  std::tuple<arma::mat, arma::uword, Actions::Action> matrix_best_action)
{
  arma::mat matrix;
  arma::uword index_of_best_estimation, value;
  std::tie(matrix, value, std::ignore) = matrix_best_action;
  index_of_best_estimation = (matrix.n_rows - 1) - value;
  std::vector<double> current_predicted_state;
  current_predicted_state.push_back(matrix(index_of_best_estimation, 0));
  current_predicted_state.push_back(matrix(index_of_best_estimation, 1));
  current_predicted_state.push_back(matrix(index_of_best_estimation, 2));
  current_predicted_state.push_back(matrix(index_of_best_estimation, 3));  
  quad_->current_predicted_state(current_predicted_state);
  double loss = std::pow((matrix(index_of_best_estimation, 0) -
                          quad_->current_state().distances_3D().at(0)),
                         2) +
                std::pow((matrix(index_of_best_estimation, 2) -
                          quad_->current_state().distances_3D().at(2)),
                         2) +
                std::pow((matrix(index_of_best_estimation, 3) -
                          quad_->current_state().height_difference()),
                         2);
  return loss;
}

template<class simulator_t>
double
AnnPredictor<simulator_t>::real_time_loss() const
{
  return real_time_loss_;
}

template<class simulator_t>
std::tuple<arma::mat, arma::uword, Actions::Action>
AnnPredictor<simulator_t>::predict(arma::mat& features)
{
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
                   mlpack::ann::RandomInitialization>
    classification_model;

  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  if (classification_) {
    mlpack::data::Load(model_path_, model_name_, classification_model, true);
  } else if (regression_) {
    mlpack::data::Load(model_path_, model_name_, regression_model, true);
  }

  /*  We need to predict the action for the follower using h(S)*/
  /*  Extract state and push it into the model with several actions */
  /*  Take the action index for the highest class
      given back by the model */
  arma::mat label;

  if (classification_) {
    classification_model.Predict(features, label);
  } else if (regression_) {
    regression_model.Predict(features, label);
  }

  /* Transpose to the original format */
  features = features.t();
  label = label.t();

  logger::logger_->info("Size of features matrix: {}", arma::size(features));
  logger::logger_->info("Feature matrix: {}", features);
  logger::logger_->info("Label matrix: {}", label);

  arma::uword value = index_of_best_action_regression(label);
  logger::logger_->info("Index of best action: {}", value);

  /*  Get the follower action now !! and store it directly */
  Actions::Action action_follower = action_.int_to_action(value);
  return make_tuple(label, value, action_follower);
}

template<class simulator_t>
Actions::Action
AnnPredictor<simulator_t>::get_predicted_action()
{
  Actions::Action predicted_follower_action;
  /*  Test the trained model using the absolute gazebo distance feature */
  arma::mat features = create_absolute_features_matrix();
  /*  Predict the next state using the above data */
  auto matrix_best_action = predict(features);
  real_time_loss_ = real_time_loss(matrix_best_action);
  std::tie(std::ignore, std::ignore, predicted_follower_action) =
    matrix_best_action;
  return predicted_follower_action;
}
