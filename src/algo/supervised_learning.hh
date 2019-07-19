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
# include "../config_ini.hh"

namespace lt = local_types;

template<class flight_controller_t,
	 class simulator_t>
class Supervised_learning
{
  
public:
  
  Supervised_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,	     
	     std::shared_ptr<simulator_t> gzs);
        
  void move_action(std::string label,
		   typename Quadcopter<simulator_t>::Action action);  
  
  void phase_two(bool random_leader_action);
  
  int highest_values(arma::mat matrix);
  
  typename Quadcopter<simulator_t>::Action randomize_action();
  
  typename Quadcopter<simulator_t>::Action 
  action_follower(arma::mat features, arma::uword index);

  arma::mat 
  features_extractor(std::vector<typename Quadcopter<simulator_t>::Action> actions);
  
  arma::mat 
  insert_absolute_features(std::vector<typename Quadcopter<simulator_t>::Action> actions);
      
  void run();

  Supervised_learning(Supervised_learning const&) = delete;  
  
  Supervised_learning(Supervised_learning &&) = default;

private:

  std::vector<typename Quadcopter<simulator_t>::Action> action_follower_ ;
  Configs configs_;
  int count_;
  DataSet data_set_;
  float decay_rate_ ;
  float discount_rate_ ;
  std::uniform_real_distribution<> distribution_;
  std::uniform_int_distribution<> distribution_int_;
  int episode_;
  float epsilon_;
  std::vector<double> flight_errors_;
  std::random_device random_dev;
  std::mt19937 generator_;
  float learning_rate_ ;
  int max_episode_ ;
  float min_epsilon_ ;
  Math_tools mtools_;      
  std::vector<std::shared_ptr<flight_controller_t>> iris_x_;
  typename Quadcopter<simulator_t>::Action saved_leader_action_;
  std::shared_ptr<simulator_t> sim_interface_;
  float speed_;
  std::vector<double> step_errors_;
  std::vector<typename Quadcopter<simulator_t>::State> states_;
  lt::triangle<double> original_dist_;
  Quadcopter<simulator_t> robot_;
  
};

# include "supervised_learning.hxx"
