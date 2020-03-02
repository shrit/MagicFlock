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
  std::vector<Actions::Action> actions = action_.all_possible_actions();

  for (int i = 0; i < 7; ++i) {
    arma::colvec col;
    col.insert_rows(col.n_rows, quad_->last_state().Data());
    col.insert_rows(col.n_rows,
                    mtools_.to_one_hot_encoding(quad_->current_action(), 7));
    col.insert_rows(col.n_rows, quad_->current_state().Data());
    col.insert_rows(col.n_rows, mtools_.to_one_hot_encoding(actions.at(i), 7));

    /*  Create a matrix of several columns, each one is added to on the end */
    features.insert_cols(features.n_cols, col);
  }
  /*  The return features need to be used in the model in order to
      give back the best action with highest score */
  return features;
}

template<class simulator_t>
template<typename State>
arma::mat
AnnPredictor<simulator_t>::create_state_matrix(State state,
                                               arma::uword matrix_size)
{
  arma::mat state_matrix;
  for (int i = 0; i < matrix_size; ++i) {
    state_matrix.insert_cols(state_matrix.n_cols, state.Data());
  }
  return state_matrix;
}

template<class simulator_t>
double
AnnPredictor<simulator_t>::compute_real_loss(const arma::mat& labels,
                                             arma::uword index)
{
  loss_vector_.clear();
  loss_vector_ = this->quad_->current_state().Data() - labels.col(index);

  this->quad_->current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class simulator_t>
double
AnnPredictor<simulator_t>::compute_absolute_loss(const arma::mat& labels,
                                                 arma::uword index)
{
  loss_vector_.clear();
  loss_vector_ =
    arma::abs(this->quad_->current_state().Data() - labels.col(index));

  this->quad_->current_loss(loss_vector_);
  return arma::sum(loss_vector_);
}

template<class simulator_t>
double
AnnPredictor<simulator_t>::compute_square_loss(const arma::mat& labels,
                                               arma::uword index)
{
  loss_vector_.clear();
  mlpack::ann::MeanSquaredError<arma::rowvec, arma::rowvec> mse;
  double error =
    mse.Forward(this->quad_->current_state().Data(), labels.col(index));

  loss_vector_ =
    arma::square(this->quad_->current_state().Data() - labels.col(index));

  this->quad_->current_loss(loss_vector_);
  return error;
}
