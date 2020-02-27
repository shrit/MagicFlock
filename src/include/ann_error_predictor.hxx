#pragma once

template<class simulator_t>
AnnErrorPredictor<simulator_t>::AnnErrorPredictor(
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
AnnErrorPredictor<simulator_t>::predict()
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path_, model_name_, regression_model, true);

  /*  We need to predict the action for the follower using h(S)*/
  /*  Extract state and push it into the model with several actions */
  /*  Take the action index for the highest class
      given back by the model */
  arma::mat features, labels;
  features = this->create_features_matrix();
  regression_model.Predict(features, labels);

  /* Transpose to the original format */
  features = features.t();
  labels = labels.t();

  logger::logger_->info("Size of error features matrix: {}",
                        arma::size(features));
  logger::logger_->info("Error feature matrix:\n {}", features);
  logger::logger_->info("Error prediction matrix:\n {}", labels);

  /* return the predicted error based on the feature vector*/
  return labels;
}

template<class simulator_t>
arma::colvec
AnnErrorPredictor<simulator_t>::predict_specific_action_error(
  Actions::Action action)
{
  arma::mat labels = predict();
  arma::uword label_index_of_best_estimation = this->action_.as_integer(action);
  arma::colvec loss_vector = labels.col(label_index_of_best_estimation);

  real_time_loss_ = arma::sum(loss_vector);
  return loss_vector;
}

template<class simulator_t>
double
AnnErrorPredictor<simulator_t>::real_time_loss() const
{
  return real_time_loss_;
}
