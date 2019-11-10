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

# include "../global.hh"

class Action {

public:

  Action();
  
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
  
  Action int_to_action(int action_value);
  
  std::vector<Action> all_possible_actions() const;

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
  
};
