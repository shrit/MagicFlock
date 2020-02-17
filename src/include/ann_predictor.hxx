#pragma once

template<class simulator_t>
AnnPredictor<simulator_t>::AnnPredictor(
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : quad_(quad)
{
  // Nothing to do here.
}

template<class simulator_t>
arma::mat
AnnPredictor<simulator_t>::create_features_matrix()
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
arma::mat
AnnPredictor<simulator_t>::create_state_matrix(arma::uword matrix_size)
{
  arma::mat state_matrix;
  arma::rowvec row;  
  for (int i = 0; i < matrix_size; ++i) {  
    row << quad_->all_state().at(0).distances_3D().at(0)
        << quad_->all_state().at(0).distances_3D().at(1)
        << quad_->all_state().at(0).distances_3D().at(2)
        << quad_->all_state().at(0).height_difference();
    state_matrix.insert_rows(0, row);
  }
  return state_matrix;
}
