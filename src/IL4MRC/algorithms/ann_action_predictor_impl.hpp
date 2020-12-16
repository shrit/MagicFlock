#pragma once

template<class QuadrotorType>
AnnActionPredictor<QuadrotorType>::AnnActionPredictor(QuadrotorType& quad)
  : AnnPredictor<QuadrotorType>(quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_action(std::string model_path,
                                           std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->action_features_matrix();

  /*
   * In this case, we should predict only actions
   * thus, we can only support continuous action case.
   */
  arma::mat labels;

  logger::logger_->info("Size of State features matrix: {}",
                        arma::size(features));
  logger::logger_->info("State data matrix:\n {}", features.t());
  regression_model.Predict(features, labels);

  typename QuadrotorType::Action best_action;
  best_action.set_action(labels);
  return best_action;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_mig_action(std::string model_path,
                                                        std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->mig_vel_action_features_matrix();

  arma::mat labels;

  logger::logger_->info("Size of Migration features matrix: {}",
                        arma::size(features));
  logger::logger_->info("Migration matrix:\n {}", features.t());
  regression_model.Predict(features, labels);
  typename QuadrotorType::Action best_action;
  best_action.set_action(labels);
  return best_action;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_cohsep_action(
  std::string model_path,
  std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->cohsep_vel_action_features_matrix();

  arma::mat labels;

  logger::logger_->info("Size of cohsep features matrix: {}",
                        arma::size(features));
  logger::logger_->info("CohSep matrix:\n {}", features.t());
  regression_model.Predict(features, labels);

  typename QuadrotorType::Action best_action;
  best_action.set_action(labels);
  return best_action;
}

