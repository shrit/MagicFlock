/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotor
 *
 *
*/
#pragma once

# include <random>
# include <utility>

/*  Armadillo includes  */
# include <armadillo>

# include "global.hh"
# include "log.hh"
# include "math_tools.hh"

class Actions {

public:

  Actions();

  /*  Add action to str printer  */
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
  
  Action
  extract_action_from_index(arma::mat features, arma::uword index);

  std::string action_to_str(Action action);
  
  Action int_to_action(int action_value);
  std::vector<Action> all_possible_actions() const;

  std::vector<Actions::Action> ActionConstructor(arma::mat values);
  
  /*  Random action generator */
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
  Math_tools mtools_;
};
