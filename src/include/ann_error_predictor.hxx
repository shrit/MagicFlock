#pragma once

template<class simulator_t>
AnnErrorPredictor<simulator_t>::AnnErrorPredictor(
  std::string full_path_to_model,
  std::string model_name,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : real_time_loss_(0)
  , model_path_(full_path_to_model)
  , model_name_(model_name)
  , quad_(quad)
{
  // Nothing to do here.
}

template<class simulator_t>
arma::mat
AnnErrorPredictor<simulator_t>::create_error_feature_vector()
{
  arma::mat features;
  arma::rowvec row;
  std::vector<Actions::Action> actions = action_.all_possible_actions();

  row << quad_->last_state().distances_3D().at(0)
      << quad_->last_state().distances_3D().at(1)
      << quad_->last_state().distances_3D().at(2)
      << quad_->last_state().height_difference()
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(0)
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(1)
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(2)
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(3)
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(4)
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(5)
      << mtools_.to_one_hot_encoding(quad_->last_action(), 7).at(6)
      << quad_->current_state().distances_3D().at(0)
      << quad_->current_state().distances_3D().at(1)
      << quad_->current_state().distances_3D().at(2)
      << quad_->current_state().height_difference()
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(0)
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(1)
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(2)
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(3)
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(4)
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(5)
      << mtools_.to_one_hot_encoding(quad_->current_action(), 7).at(6);

  /*  Create a matrix of several rows, each one is added to on the top */
  features.insert_rows(0, row);

  /*  We need to transpose the matrix, since mlpack is column major */
  features = features.t();
  /*  The return features need to be used in the model in order to
      give back the best action with highest score */
  return features;
}

template<class simulator_t>
arma::mat
AnnErrorPredictor<simulator_t>::predict_error(arma::mat& features)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path_, model_name_, regression_model, true);

  /*  We need to predict the action for the follower using h(S)*/
  /*  Extract state and push it into the model with several actions */
  /*  Take the action index for the highest class
      given back by the model */
  arma::mat label;
  regression_model.Predict(features, label);

  /* Transpose to the original format */
  features = features.t();
  label = label.t();

  logger::logger_->info("Size of features matrix: {}", arma::size(features));
  logger::logger_->info("Feature matrix: {}", features);
  logger::logger_->info("Label matrix: {}", label);

  /* return the predicted error based on the feature vector*/
  return label;
}
