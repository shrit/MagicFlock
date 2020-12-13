#pragma once

template<class QuadrotorType>
AnnActionPredictor<QuadrotorType>::AnnActionPredictor(QuadrotorType& quad)
  : AnnPredictor<QuadrotorType>(quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnActionPredictor<QuadrotorType>::predict(std::string model_path,
                                           std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->create_features_matrix();

  /*
   * In this case, we should predict only actions
   * thus, we can only support continuous action case.
   */
  arma::mat labels;
  if constexpr (std::is_same<typename QuadrotorType::Action,
                             ContinuousActions>::value) {
    labels.clear();
    logger::logger_->info("Size of State features matrix: {}",
                          arma::size(features));
    logger::logger_->info("State data matrix:\n {}", features.t());
    regression_model.Predict(features, labels);
  }

  return labels;
}

template<class QuadrotorType>
arma::mat
AnnActionPredictor<QuadrotorType>::predict_mig_velocity(std::string model_path,
                                                        std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->create_mig_vel_features_matrix();

  arma::mat labels;
  if constexpr (std::is_same<typename QuadrotorType::Action,
                             ContinuousActions>::value) {
    labels.clear();
    logger::logger_->info("Size of Migration features matrix: {}",
                          arma::size(features));
    logger::logger_->info("Migration matrix:\n {}", features.t());
    regression_model.Predict(features, labels);
  }
  return labels;
}

template<class QuadrotorType>
arma::mat
AnnActionPredictor<QuadrotorType>::predict_cohsep_velocity(
  std::string model_path,
  std::string model_name)
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path, model_name, regression_model, true);
  arma::mat features = this->create_cohsep_vel_features_matrix();

  arma::mat labels;
  if constexpr (std::is_same<typename QuadrotorType::Action,
                             ContinuousActions>::value) {
    labels.clear();
    logger::logger_->info("Size of cohsep features matrix: {}",
                          arma::size(features));
    logger::logger_->info("CohSep matrix:\n {}", features.t());
    regression_model.Predict(features, labels);
  }
  return labels;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_action(std::string model_path,
                                                         std::string model_name)
{
  /* Predict the action using the above data */
  typename QuadrotorType::Action best_action;
  best_action.set_action(predict(model_path, model_name));
  return best_action;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_mig_action(
  std::string model_path,
  std::string model_name)
{
  /* Predict the action using the above data */
  typename QuadrotorType::Action best_action;
  best_action.set_action(predict_mig_velocity(model_path, model_name));
  return best_action;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_cohsep_action(
  std::string model_path,
  std::string model_name)
{
  /* Predict the action using the above data */
  typename QuadrotorType::Action best_action;
  best_action.set_action(predict_cohsep_velocity(model_path, model_name));
  return best_action;
}
