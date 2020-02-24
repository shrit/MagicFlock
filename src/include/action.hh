/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotor
 *
 *
 */
#pragma once

#include <random>
#include <utility>

#include "math_tools.hh"

class Actions
{

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

  template<typename Enumeration>
  auto as_integer(Enumeration const value) ->
    typename std::underlying_type<Enumeration>::type
  {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
  }


  Action extract_action_from_index(arma::mat features, arma::uword index);

  std::string action_to_str(Action action);

  Action int_to_action(int action_value);
  std::vector<Action> all_possible_actions() const;

  std::vector<Actions::Action> ActionConstructor(arma::mat values);

  /*  Random action generator */
  Action random_action_generator();

  Action random_action_generator_with_only_opposed_condition(Action action);

  Action random_action_generator_with_all_conditions(Action action);

  std::tuple<Actions::Action, Actions::Action>
  generate_followers_action_using_distance(double distance_t_1_b,
                                           double distance_t_b,
                                           double distance_t_1_c,
                                           double distance_t_c,
                                           double alti_diff_t);

  Actions::Action undo_action(Actions::Action action);
  Actions::Action pair_action_bob(Actions::Action action);
  Actions::Action pair_action_charlie(Actions::Action action);

  Action generate_follower_action_using_oracle(double distances_t_1_b,
                                               double distances_t_b,
                                               double distance_t_1_c,
                                               double distance_t_c,
                                               double alti_diff_t);

  Action generate_leader_action(bool change_leader_action,
                                bool stop_going_down,
                                Action last_action,
                                Action current_action);

  Action validate_leader_action(double distance_to_b,
                                double distance_to_b_1,
                                double distance_to_c,
                                double distance_to_c_1,
                                Action current_action);

  Action validate_followers_action(std::vector<double> current_distances,
                                   std::vector<double> last_distances,
                                   Action before_2_last_action,
                                   Action current_action);
  double generate_real_random();

protected:
  std::uniform_int_distribution<> distribution_int_;
  std::uniform_real_distribution<> distribution_real_;
  std::random_device random_dev;
  std::mt19937 generator_;

private:
  std::vector<Action> possible_actions_ = { Action::forward, Action::backward,
                                            Action::left,    Action::right,
                                            Action::up,      Action::down,
                                            Action::NoMove };

  Math_tools mtools_;
};
