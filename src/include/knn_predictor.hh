#pragma once

/*  Standard C++ includes  */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <tuple>
#include <vector>

/* MLPack includes */
#include <mlpack/core.hpp>
#include <mlpack/methods/neighbor_search/neighbor_search.hpp>

/* local includes  */
#include "data_set.hh"
#include "distance_state.hh"
#include "global.hh"
#include "log.hh"
#include "math_tools.hh"
#include "quadrotor.hh"

namespace lt = local_types;
using namespace ILMR;

template<class simulator_t>
class KnnPredictor
{
public:
  KnnPredictor(
    std::string dataset_file,
    typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  void predict(int knn_neighbors);
  Actions::Action get_predicted_action() const;

  std::vector<double> estimate_action_from_distance(arma::mat& matrix);  
  int index_of_best_action_regression(arma::mat& matrix);

  KnnPredictor(KnnPredictor const&) = delete;
  KnnPredictor(KnnPredictor&&) = default;

private:
  DataSet<simulator_t> dataset_;
  Math_tools mtools_;
  Actions::Action predicted_follower_action_;
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;
};

#include "knn_predictor.hxx"
