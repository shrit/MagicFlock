#pragma once

template<class QuadrotorType>
AnnPredictor<QuadrotorType>::AnnPredictor(const QuadrotorType& quad)
  : quad_(quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
arma::mat
AnnPredictor<QuadrotorType>::create_features_matrix()
{
  arma::mat features;

  if constexpr (std::is_same<typename QuadrotorType::Action,
                             ContinuousActions>::value) {
    features.insert_rows(features.n_rows, quad_.current_state().Data());
    features.insert_rows(features.n_rows, quad_.current_action().Data());

  } else if constexpr (std::is_same<typename QuadrotorType::Action,
                                    DiscretActions>::value) {

    std::vector<typename QuadrotorType::Action> actions =
      action_.all_possible_actions();

    for (int i = 0; i < 7; ++i) {
      arma::colvec col;
      col.insert_rows(col.n_rows, quad_.last_state().Data());
      col.insert_rows(col.n_rows, quad_.current_action().Data());
      col.insert_rows(col.n_rows, quad_.current_state().Data());
      col.insert_rows(col.n_rows, actions.at(i).Data());

      /*  Create a matrix of several columns, each one is added to on the end */
      features.insert_cols(features.n_cols, col);
    }
  }
  /*  The return features need to be used in the model in order to
      give back the best action with highest score */
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
