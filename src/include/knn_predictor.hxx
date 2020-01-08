#pragma once

#include "knn_predictor.hh"

template<class simulator_t>
KnnPredictor<simulator_t>::KnnPredictor(
  std::string dataset_file,
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : quad_(quad)
{
  dataset_.parse_dataset_file(dataset_file);
}

template<class simulator_t>
void
KnnPredictor<simulator_t>::predict(int knn_neighbors)
{
  arma::mat s_a_s_t_1 = dataset_.s_a_s_t_1();
  arma::mat a_t_1 = dataset_.at_1_mat();
  arma::mat s_t_2 = dataset_.s_t_2_mat();
  std::vector<std::tuple<double, int>> dist_ref;

  arma::mat query = dataset_.conv_state_action_state_to_arma_state(
    quad_->last_state(), quad_->current_action(), quad_->current_state());

  mlpack::neighbor::NeighborSearch<mlpack::neighbor::NearestNeighborSort,
                                   mlpack::metric::EuclideanDistance>
    knn(s_a_s_t_1);
  arma::Mat<size_t> close_neighbors;
  arma::mat distances;

  knn.Search(query, 4, close_neighbors, distances);
  logger::logger_->debug("Closest neigh neibors indices are: {} ", close_neighbors);
  logger::logger_->debug("distances to neibors are: {} ", distances);

  arma::mat result_state = dataset_.submat_using_indices(s_t_2, distances);
  arma::uword value = index_of_best_action_regression(result_state);
  logger::logger_->debug("Index of best action: {}", value);

  predicted_follower_action_ = a_t_1.at(value);
}

template<class simulator_t>
int
KnnPredictor<simulator_t>::index_of_best_action_regression(arma::mat& matrix)
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
  double d1, d2;
  double original_d1 = 3;
  double original_d2 = 3;
  double height_diff;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    d1 = std::fabs(original_d1 - matrix(i, 0));
    d2 = std::fabs(original_d2 - matrix(i, 1));
    height_diff =
      std::fabs(quad_->current_state().height_difference() - matrix(i, 2));
    sum_of_distances.push_back(d1 + d2 + height_diff);
  }
  return sum_of_distances;
}

template<class simulator_t>
Actions::Action
KnnPredictor<simulator_t>::get_predicted_action() const
{
  return predicted_follower_action_;
}
