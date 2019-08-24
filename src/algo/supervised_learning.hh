#pragma once

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <numeric>
# include <random>
# include <thread>
# include <vector>

/*  MLPack ncludes */
#include <mlpack/core.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>

/*  Armadillo includes  */
# include <armadillo>

/* local includes  */
# include "../data_set.hh"
# include "../global.hh"
# include "../gazebo.hh"
# include "../log.hh"
# include "../math_tools.hh"
# include "quadcopter.hh"
# include "../swarm_device.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class Supervised_learning
{

public:

  Supervised_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
		      std::shared_ptr<simulator_t> gzs);
  
  int index_of_best_action(arma::mat matrix);

  arma::mat
  insert_absolute_features(std::vector<Quadcopter::Action> actions);

  arma::mat
  insert_estimated_features(std::vector<Quadcopter::Action> actions);
  
  void run();

  void phase_two(bool random_leader_action);

  Supervised_learning(Supervised_learning const&) = delete;

  Supervised_learning(Supervised_learning &&) = default;

private:

  std::vector<Quadcopter::Action> action_follower_ ;
  int count_;
  DataSet data_set_;
  int episode_;
  std::vector<double> flight_errors_;
  int max_episode_ ;
  Math_tools mtools_;
  Quadcopter::Action saved_leader_action_;
  std::shared_ptr<simulator_t> sim_interface_;
  std::vector<double> step_errors_;
  std::vector<Quadcopter::State<simulator_t>> states_;
  std::vector<int>  time_step_vector_;
  lt::triangle<double> original_dist_;
  Quadcopter robot_;
  SwarmDevice<flight_controller_t> swarm_;
  arma::rowvec controller_predictions_;
};

# include "supervised_learning.hxx"
