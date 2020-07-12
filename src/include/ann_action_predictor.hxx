#pragma once

template<class QuadrotorType>
AnnStatePredictor<QuadrotorType>::AnnStatePredictor(
  std::string full_path_to_model,
  std::string model_name,
  QuadrotorType& quad)
  : AnnPredictor<QuadrotorType>(quad)
  , model_path_(full_path_to_model)
  , model_name_(model_name)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnActionPredictor<QuadrotorType>::predict()
{
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
    regression_model;

  mlpack::data::Load(model_path_, model_name_, regression_model, true);
  arma::mat features = this->create_features_matrix();

  /*
   * In this case, when we would like to predict only actions
   * we support only continuous action case.
   */
  if constexpr (std::is_same<typename QuadrotorType::Action,
                             ContinuousActions>::value) {
    labels_.clear();
    regression_model.Predict(features, labels_);
  }

  return labels_;
}

template<class QuadrotorType>
typename QuadrotorType::Action
AnnActionPredictor<QuadrotorType>::best_predicted_action()
{
  /* Predict the action using the above data */
  best_action_.action() = predict();
  return best_action_;
}

