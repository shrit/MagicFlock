#pragma once

# include "predictor.hh"

Predictor::Predictor()
{}

/* Estimate the features (distances) using propagation model from RSSI */
arma::mat Predictor::
create_estimated_features_matrix(std::vector<Quadcopter::Action> actions)
{
  arma::mat features;
  auto it_state = states_.rbegin();
  it_state = std::next(it_state, 1);
  arma::rowvec row;
  for (int i = 0; i < 7; ++i) {
    /*  State */
    row << (*it_state).distances_3D().f1
	<< (*it_state).distances_3D().f2
	<< (*it_state).height_difference()
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(0)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(1)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(2)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(3)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(4)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(5)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(6)
	<< states_.back().distances_3D().f1
	<< states_.back().distances_3D().f2
	<< states_.back().height_difference()
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

arma::mat Predictor::
create_absolute_features_matrix(std::vector<Quadcopter::Action> actions)
{
  arma::mat features;
  arma::rowvec row;
  auto it = states_.rbegin();
  it = std::next(it, 1);
  for (int i = 0; i < 7; ++i) {
    /*  State */
    row << (*it).distances_3D().f1
	<< (*it).distances_3D().f3
	<< (*it).height_difference()
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(0)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(1)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(2)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(3)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(4)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(5)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(6)
	<< states_.back().distances_3D().f1
	<< states_.back().distances_3D().f3
	<< states_.back().height_difference()
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

int Predictor::
index_of_best_action_classification(arma::mat matrix)
{
  int value = 0;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    value = arma::index_max(matrix.col(0));
  }
  return value;
}

int Predictor::
index_of_best_action_regression(arma::mat matrix)
{
  std::vector<double> distances = estimate_action_from_distance(matrix);
  std::reverse(distances.begin(), distances.end());
  LogInfo() << "Sum of distances: " << distances;
  int value =
    std::min_element(distances.begin(),
		     distances.end()) - distances.begin();
  return value;
}

std::vector<double> Predictor::
estimate_action_from_distance(arma::mat matrix)
{
  std::vector<double> sum_of_distances;
  lt::triangle<double> distances;
  double height_diff;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    /*  0 index is the height, not considered yet */
    /*  Consider only f1 and f2 */
    distances.f1 = std::fabs(original_dist_.f1 - matrix(i, 0));
    distances.f2 = std::fabs(original_dist_.f2 - matrix(i, 1));
    height_diff = std::fabs(height_diff_ - matrix(i, 2));
    sum_of_distances.push_back(distances.f1 + distances.f2 + height_diff);
  }
  return sum_of_distances;
}

double Predictor::
real_time_loss(std::tuple<arma::mat, arma::uword> matrix_best_action)
{
  arma::mat matrix;
  arma::uword index_of_best_estimation;
  std::tie(matrix, index_of_best_estimation) = matrix_best_action;
  double loss = std::pow((matrix(index_of_best_estimation, 1) - states_.back().distances_3D().f1), 2) +
    std::pow((matrix(index_of_best_estimation, 2) - states_.back().distances_3D().f2), 2);
  return loss;
}

std::tuple<arma::mat, arma::uword> Predictor::
predict(arma::mat features)
{
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> classification_model;
  
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
		   mlpack::ann::RandomInitialization> regression_model;
  
  if (classification_) {  
    mlpack::data::Load("model.txt", "model", classification_model, true);
  } else if(regression_) {
    mlpack::data::Load("model.txt", "model", regression_model, true);
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

  LogInfo() << "Size of features: " << arma::size(features);
  LogInfo() << features;
  LogInfo() << label;

  arma::uword value = index_of_best_action_regression(label);
  LogInfo() << value;

  /*  Get the follower action now !! and store it directly */
  action_follower_.push_back(robot_.int_to_action(value));  
  return make_tuple(label, value);
}
