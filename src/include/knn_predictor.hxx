#pragma once

#include "knn_predictor.hh"

template<class simulator_t>
KnnPredictor<simulator_t>::KnnPredictor(
  std::string dataset_file,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : quad_(quad)
{
  dataset_.parse_dataset_file(dataset_file);
  dataset_.load_knn_dataset(dataset_file);
}

template<class simulator_t>
void
KnnPredictor<simulator_t>::predict(int knn_neighbors)
{
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
  logger::logger_->info("Closest neighbors indices are: {} ", close_neighbors);
  logger::logger_->info("distances to neighbors are: {} ", distances);
  logger::logger_->info("State matrix size: {}", arma::size(s_t_2));
  s_t_2 = s_t_2.t(); // transpose the matrix into row major
  logger::logger_->info("Neighbor matrix size: {}",
                        arma::size(close_neighbors));
  arma::mat result_state =
    dataset_.submat_using_indices(s_t_2, close_neighbors);
  arma::uword value = index_of_best_state(result_state);
  logger::logger_->debug("Index of best state: {}", value);
  a_t_1 = a_t_1.t();
  arma::rowvec action =
    a_t_1(arma::span(close_neighbors(value, 0), close_neighbors(value, 0)),
          arma::span(0, a_t_1.n_cols - 1));
  logger::logger_->info("action before converting: {}", action);
  std::vector action_vec = mtools_.to_std_vector(action);
  std::reverse(action_vec.begin(), action_vec.end());
  Actions actions;
  predicted_follower_action_ =
    actions.int_to_action(mtools_.from_one_hot_encoding(action_vec));
}

template<class simulator_t>
int
KnnPredictor<simulator_t>::index_of_best_state(arma::mat& matrix)
{
  std::vector<double> distances = estimate_action_from_distance(matrix);
  std::reverse(distances.begin(), distances.end());
  logger::logger_->info("Sum of distances: {}", distances);
  int value =
    std::min_element(distances.begin(), distances.end()) - distances.begin();
  return value;
}

template<class simulator_t>
std::vector<double>
KnnPredictor<simulator_t>::estimate_action_from_distance(arma::mat& matrix)
{
  std::vector<double> sum_of_distances;
  double d1, d2, d3;
  double height_diff;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    d1 =
      std::fabs(quad_->all_states().at(0).distances_3D().at(0) - matrix(i, 0));
    d2 =
      std::fabs(quad_->all_states().at(0).distances_3D().at(1) - matrix(i, 1));
    d3 =
      std::fabs(quad_->all_states().at(0).distances_3D().at(2) - matrix(i, 2));
    height_diff =
      std::fabs(quad_->current_state().height_difference() - matrix(i, 3));
    sum_of_distances.push_back(d1 + d3 + height_diff);
  }
  return sum_of_distances;
}

template<class simulator_t>
Actions::Action
KnnPredictor<simulator_t>::get_predicted_action() const
{
  return predicted_follower_action_;
}
