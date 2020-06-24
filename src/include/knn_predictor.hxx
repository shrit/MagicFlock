#pragma once

template<class QuadrotorType>
KnnPredictor<QuadrotorType>::KnnPredictor(
  std::string dataset_file,
  const QuadrotorType& quad)
  : quad_(quad)
{
  dataset_.parse_dataset_file(dataset_file);
  dataset_.load_knn_dataset(dataset_file);
}

template<class QuadrotorType>
void
KnnPredictor<QuadrotorType>::predict(int knn_neighbors)
{
  /* All of the following matrices are col major*/
  arma::mat s_a_s_t_1 = dataset_.s_a_s_t_1_mat();
  arma::mat a_t_1 = dataset_.at_1_mat();
  arma::mat s_t_2 = dataset_.st_2_mat();

  arma::mat query = dataset_.conv_state_action_state_to_arma(
    quad_->last_state(), quad_->current_action(), quad_->current_state());

  mlpack::neighbor::NeighborSearch<mlpack::neighbor::NearestNeighborSort,
                                   mlpack::metric::EuclideanDistance>
    knn(s_a_s_t_1);
  arma::Mat<size_t> close_neighbors;
  arma::mat distances;

  knn.Search(query, knn_neighbors, close_neighbors, distances);
  logger::logger_->info("Closest neighbors indices are:\n {} ",
                        close_neighbors);
  logger::logger_->info("distances to neighbors are:\n {} ", distances);
  logger::logger_->info("State matrix size: {}", arma::size(s_t_2));

  logger::logger_->info("Neighbor matrix size:\n {}",
                        arma::size(close_neighbors));

  // Transpose the state_2 matrix into row major to recover the result states.
  arma::mat result_state =
    dataset_.submat_using_indices(s_t_2.t(), close_neighbors);

  result_state = result_state.t();

  arma::mat original_state_matrix =
    this->create_state_matrix(this->quad_->all_states().at(0), labels_.n_cols);
  Argmin<arma::mat, arma::uword> argmin(original_state_matrix, result_state, 1);

  arma::uword value = argmin.min_index();
  logger::logger_->debug("Index of best state: {}", value);
  /* Need to be tested, need to re cut the following matrix only one value in
   * this case*/
  arma::colvec action =
    a_t_1(arma::span(close_neighbors(value, 0), close_neighbors(value, 0)),
          arma::span(0, a_t_1.n_cols - 1));

  logger::logger_->info("action vector: {}", action);

  predicted_follower_action_ =
    actions.int_to_action(one_hot_.from_one_hot_encoding(action));
}

template<class QuadrotorType>
Actions::Action
KnnPredictor<QuadrotorType>::get_predicted_action() const
{
  return predicted_follower_action_;
}
