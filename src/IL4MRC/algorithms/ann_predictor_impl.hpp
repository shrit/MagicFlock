#pragma once

template<class QuadrotorType>
AnnPredictor<QuadrotorType>::AnnPredictor(QuadrotorType& quad)
  : quad_(quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::state_features_matrix()
{
  arma::mat features;

  // Uncomment the following if you want to predict the state predictor
  arma::mat actions = action_.all_possible_actions();
  for (int i = 0; i < actions.n_cols; ++i) {
    arma::colvec col;
    col.insert_rows(col.n_rows, quad_.before_2_last_state().Data());
    col.insert_rows(col.n_rows, quad_.before_last_action().Data());
    col.insert_rows(col.n_rows, quad_.before_last_state().Data());
    col.insert_rows(col.n_rows, quad_.last_action().Data());
    col.insert_rows(col.n_rows, quad_.last_state().data());
    col.insert_rows(col.n_rows, quad_.current_action().data());
    col.insert_rows(col.n_rows, quad_.current_state().data());
    col.insert_rows(col.n_rows, actions.col(i));

    /*  create a matrix of several columns, each one is added to on the end
     */
    features.insert_cols(features.n_cols, col);
  }
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::action_features_matrix()
{
  arma::mat features;

  // Uncomment the following test the action predictor
  arma::colvec col;
  col.insert_rows(col.n_rows, quad_.before_2_last_state().Data());
  col.insert_rows(col.n_rows, quad_.before_last_action().Data());
  col.insert_rows(col.n_rows, quad_.before_last_state().Data());
  col.insert_rows(col.n_rows, quad_.last_action().Data());
  col.insert_rows(col.n_rows, quad_.last_state().Data());
  col.insert_rows(col.n_rows, quad_.current_action().Data());
  col.insert_rows(col.n_rows, quad_.current_state().Data());
  features.insert_cols(features.n_cols, col);

  /*  The return features need to be used in the model in order to
    give back the best action with highest score */
  return features;
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::cohsep_vel_state_features_matrix()
{
  arma::mat features;
  arma::mat actions = action_.all_possible_actions();
  for (int i = 0; i < actions.n_cols; ++i) {
    arma::colvec col;
    col.insert_rows(col.n_rows, quad_.before_2_last_state().followers_data());
    col.insert_rows(col.n_rows, quad_.before_last_action().followers_data());
    col.insert_rows(col.n_rows, quad_.before_last_state().followers_data());
    col.insert_rows(col.n_rows, quad_.last_action().followers_data());
    col.insert_rows(col.n_rows, quad_.last_state().followers_data());
    col.insert_rows(col.n_rows, quad_.current_action().followers_data());
    col.insert_rows(col.n_rows, quad_.current_state().followers_data());
    col.insert_rows(col.n_rows, actions.col(i));
    features.insert_cols(features.n_cols, col);
  }
  return features;
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::cohsep_vel_action_features_matrix()
{
  arma::mat features;
  arma::colvec col;
  col.insert_rows(col.n_rows, quad_.before_2_last_state().followers_data());
  col.insert_rows(col.n_rows, quad_.before_last_action().followers_data());
  col.insert_rows(col.n_rows, quad_.before_last_state().followers_data());
  col.insert_rows(col.n_rows, quad_.last_action().followers_data());
  col.insert_rows(col.n_rows, quad_.last_state().followers_data());
  col.insert_rows(col.n_rows, quad_.current_action().followers_data());
  col.insert_rows(col.n_rows, quad_.current_state().followers_data());
  features.insert_cols(features.n_cols, col);
  return features;
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::mig_vel_state_features_matrix()
{
  arma::mat features;
  arma::mat actions = action_.all_possible_actions();
  for (int i = 0; i < actions.n_cols; ++i) {
    arma::colvec col;
    col.insert_rows(col.n_rows, quad_.before_2_last_state().leader_data());
    col.insert_rows(col.n_rows, quad_.before_last_action().leader_data());
    col.insert_rows(col.n_rows, quad_.before_last_state().leader_data());
    col.insert_rows(col.n_rows, quad_.last_action().leader_data());
    col.insert_rows(col.n_rows, quad_.last_state().leader_data());
    col.insert_rows(col.n_rows, quad_.current_action().leader_data());
    col.insert_rows(col.n_rows, quad_.current_state().leader_data());
    col.insert_rows(col.n_rows, actions.col(i));
    features.insert_cols(features.n_cols, col);
  }
  return features;
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::mig_vel_action_features_matrix()
{
  arma::mat features;
  arma::colvec col;
  col.insert_rows(col.n_rows, quad_.before_2_last_state().leader_data());
  col.insert_rows(col.n_rows, quad_.before_last_action().leader_data());
  col.insert_rows(col.n_rows, quad_.before_last_state().leader_data());
  col.insert_rows(col.n_rows, quad_.last_action().leader_data());
  col.insert_rows(col.n_rows, quad_.last_state().leader_data());
  col.insert_rows(col.n_rows, quad_.current_action().leader_data());
  col.insert_rows(col.n_rows, quad_.current_state().leader_data());
  features.insert_cols(features.n_cols, col);
  return features;
}

template<class QuadrotorType>
template<typename State>
arma::mat
AnnPredictor<QuadrotorType>::create_state_matrix(State state,
                                                 arma::uword matrix_size)
{
  arma::mat state_matrix;
  for (int i = 0; i < matrix_size; ++i) {
    state_matrix.insert_cols(state_matrix.n_cols, state.Data());
  }
  return state_matrix;
}

template<class QuadrotorType>
double
AnnPredictor<QuadrotorType>::compute_real_loss(const arma::mat& labels,
                                               arma::uword index)
{
  loss_vector_.clear();
  loss_vector_ = this->quad_.current_state().Data() - labels.col(index);

  this->quad_.current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class QuadrotorType>
double
AnnPredictor<QuadrotorType>::compute_absolute_loss(const arma::mat& labels,
                                                   arma::uword index)
{
  loss_vector_.clear();
  loss_vector_ =
    arma::abs(this->quad_.current_state().Data() - labels.col(index));

  this->quad_.current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class QuadrotorType>
double
AnnPredictor<QuadrotorType>::compute_square_loss(const arma::mat& labels,
                                                 arma::uword index)
{
  loss_vector_.clear();
  mlpack::ann::MeanSquaredError<arma::rowvec, arma::rowvec> mse;
  double error =
    mse.Forward(this->quad_.current_state().Data(), labels.col(index));

  loss_vector_ =
    arma::square(this->quad_.current_state().Data() - labels.col(index));

  this->quad_.current_loss(loss_vector_);
  return error;
}
