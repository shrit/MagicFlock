#pragma once

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <numeric>
# include <tuple>
# include <vector>

/*  Armadillo includes  */
# include <armadillo>

/* local includes  */
# include "data_set.hh"
# include "distance_state.hh"
# include "global.hh"
# include "log.hh"
# include "math_tools.hh"
# include "quadrotor.hh"

namespace lt = local_types;

template<class simulator_t>
class StatisticalPredictor
{
public:
  StatisticalPredictor(std::string dataset_file,
		       typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  void predict();
  Actions::Action get_predicted_action() const;

  StatisticalPredictor(StatisticalPredictor const&) = delete;
  StatisticalPredictor(StatisticalPredictor &&) = default;

private:
  DataSet<simulator_t> dataset_;
  Math_tools mtools_;
  Actions::Action predicted_follower_action_;
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;
};

# include "statistical_predictor.hxx"
