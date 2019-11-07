#pragma once

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <numeric>
# include <random>
# include <thread>
# include <tuple>
# include <vector>

/*  MLPack includes */
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

  std::vector<double>
  estimate_action_from_distance(arma::mat matrix);

  void generate_trajectory_using_model(bool random_leader_action,
				       bool stop_down_action);
  
  int index_of_best_action_classification(arma::mat matrix);
  int index_of_best_action_regression(arma::mat matrix);
  
  arma::mat
  create_absolute_features_matrix(std::vector<Quadcopter::Action> actions);

  arma::mat
  create_estimated_features_matrix(std::vector<Quadcopter::Action> actions);

  std::tuple<arma::mat, arma::uword> predictor(arma::mat features);
  
  double real_time_loss(std::tuple<arma::mat, arma::uword> matrix_best_action);
  
  void run(const Settings& settings);

  Supervised_learning(Supervised_learning const&) = delete;
  Supervised_learning(Supervised_learning &&) = default;

private:

  std::vector<Quadcopter::Action> action_follower_;
  int count_;
  bool classification_;
  DataSet data_set_;
  int episode_;
  std::vector<double> flight_errors_;
  int max_episode_ ;
  Math_tools mtools_;
  bool regression_;
  Quadcopter::Action saved_leader_action_;
  std::shared_ptr<simulator_t> sim_interface_;
  std::vector<double> step_errors_;
  std::vector<Quadcopter::State<simulator_t>> states_;
  std::vector<int>  time_step_vector_;
  lt::triangle<double> original_dist_;
  double height_diff_;
  Quadcopter robot_;
  SwarmDevice<flight_controller_t> swarm_;
  bool stop_episode_;
  Quadcopter::Action action_leader_;
};

# include "supervised_learning.hxx"
