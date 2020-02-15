#pragma once

template<class simulator_t>
AnnErrorPredictor<simulator_t>::AnnErrorPredictor(
  std::string full_path_to_model,
  std::string model_name,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : AnnPredictor(quad)
  , real_time_loss_(0)
  , model_path_(full_path_to_model)
  , model_name_(model_name)
{
  // Nothing to do here.
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
