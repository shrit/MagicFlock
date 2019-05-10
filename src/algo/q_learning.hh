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
# include "../global.hh"
# include "../gazebo.hh"
# include "../log.hh"
# include "../math_tools.hh"
# include "quadcopter.hh"

namespace lt = local_types;

template<class flight_controller_t>
class Q_learning
{
  
public:
  
  Q_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,	     
	     std::shared_ptr<Gazebo> gzs);
        
  void move_action(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
		   std::string label,
		   float speed,
		   Quadcopter<Gazebo>::Action action);  
  
  void phase_two(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
		 float speed,                                             
		 std::shared_ptr<Gazebo> gzs,                             
		 bool random_leader_action);

  int highest_values(arma::mat matrix);
  
  Quadcopter<Gazebo>::Action randomize_action();
  
  Quadcopter<Gazebo>::Action 
  action_follower(arma::mat features, arma::uword index);
  
  arma::mat 
  insert_features(std::vector<Quadcopter<Gazebo>::Action> actions);
      
  void run_episods(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
		   float speed,
		   std::shared_ptr<Gazebo> gzs);

  Q_learning(Q_learning const&) = delete;  
  
  Q_learning(Q_learning &&) = default;

private:

  int count_;
  float decay_rate_ ;   
  float discount_rate_ ;
  std::uniform_real_distribution<> distribution_;
  std::uniform_int_distribution<> distribution_int_;
  int episode_ ;
  float epsilon_ ;
  std::random_device random_dev;  
  std::mt19937 generator_;  
  float learning_rate_ ;
  int max_episode_ ;  
  float min_epsilon_ ;  
  
  std::vector<Quadcopter<Gazebo>::State> states_;

  Math_tools mtools_;
      
  std::vector<Quadcopter<Gazebo>::Action> action_follower_ ;
  
  Quadcopter<Gazebo>::Action saved_leader_action_;  
  
};

# include "q_learning.hxx"
