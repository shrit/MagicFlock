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
#include <mlpack/core/metrics/lmetric.hpp>
#include <mlpack/core/util/prefixedoutstream.hpp>
#include <mlpack/methods/neighbor_search/neighbor_search.hpp>

/* local includes  */
#include "dataset.hpp"
#include "one_hot_encoding.hpp"

using namespace ILMR;

template<class QuadrotorType>
class KnnPredictor
{
public:
  KnnPredictor(std::string dataset_file,
               const QuadrotorType& quad);

  void predict(int knn_neighbors);
  Actions::Action get_predicted_action() const;

  std::vector<double> estimate_action_from_distance(arma::mat& matrix);
  int index_of_best_state(arma::mat& matrix);

  KnnPredictor(KnnPredictor const&) = delete;
  KnnPredictor(KnnPredictor&&) = default;

private:
  OneHotEncoding one_hot_;
  DataSet dataset_;
  DiscretActions::Action predicted_follower_action_;
};

#include "knn_predictor_impl.hpp"
