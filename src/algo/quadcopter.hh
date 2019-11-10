/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 *
*/
#pragma once

# include <random>
# include <utility>

/*  Armadillo includes  */
# include <armadillo>

# include "../data_set.hh"
# include "../global.hh"
# include "../math_tools.hh"
# include "propagation_model.hh"

namespace lt = local_types;

template <class simulator_t>  
class Quadrotor {

public:

  Quadrotor();

  enum class Action
    {
     forward,
     backward,
     left,
     right,
     up,
     down,
     NoMove,
     Unknown,
    };

  enum class Reward
    {
     very_good,
     good,
     bad,
     very_bad,
     Unknown,
    };
  
  void init();

  Reward action_evaluator(const lt::triangle<double>& old_dist,
			  const lt::triangle<double>& new_dist);
        
  Action
  action_follower(arma::mat features, arma::uword index);

  Action int_to_action(int action_value);
  
  void
  save_controller_count(double value);

  std::vector<Action> possible_actions() const;

  Action random_action_generator();

  Action 
  random_action_generator_with_only_opposed_condition(Action action);

  Action
  random_action_generator_with_all_conditions(Action action);

private:

  std::vector<Action> possible_actions_ = {Action::forward,
					   Action::backward,
					   Action::left,
					   Action::right,
					   Action::up,
					   Action::down,
					   Action::NoMove};
  
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;
  std::mt19937 generator_;
  DataSet data_set_;

  Action current_action_;
  Action last_action_;
  std::vector<Action> all_actions_;
 
  State current_state_; 
  std::vector<State> all_states_;

  unsigned int id; /* Quadrotor id */
  std::string name; /* QUadrotor name */

};

# include "quadcopter.hxx"
