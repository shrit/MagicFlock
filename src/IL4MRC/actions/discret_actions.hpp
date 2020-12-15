/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotor
 *
 */
#pragma once

#include <random>
#include <tuple>
#include <utility>

#include <IL4MRC/metrics/one_hot_encoding.hpp>
#include <mlpack/prereqs.hpp>

class DiscretActions
{

public:
  DiscretActions();

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

  Action action() const;
  Action& action();

  arma::colvec Data();

  template<typename Enumeration>
  auto as_integer(Enumeration const value) ->
    typename std::underlying_type<Enumeration>::type
  {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
  }

  std::string action_to_str(Action action);

  Action int_to_action(int action_value);
  std::vector<Action> all_possible_actions() const;

  /*  Random action generator */
  Action random_action_generator();

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

  Action action_;
  OneHotEncoding one_hot_;
};
