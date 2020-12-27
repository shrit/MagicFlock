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

#include <MagicFlock/metrics/one_hot_encoding.hpp>
#include <mlpack/prereqs.hpp>

class DiscretActions
{

public:
  DiscretActions();

  ignition::math::Vector3d action() const;

  ignition::math::Vector3d& action();

  ignition::math::Vector3d leader_action() const;

  ignition::math::Vector3d& leader_action();

  ignition::math::Vector3d followers_action() const;

  ignition::math::Vector3d& followers_action();

  void set_action(arma::colvec data);

  arma::mat all_possible_actions();

  arma::colvec Data();
  arma::colvec leader_data();
  arma::colvec followers_data();

  template<typename Enumeration>
  auto as_integer(Enumeration const value) ->
    typename std::underlying_type<Enumeration>::type
  {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
  }

  int action_to_int(ignition::math::Vector3d action);
  DiscretActions int_to_action(arma::uword index);

  /*  Random action generator */
  int random_action_generator();

protected:
  ignition::math::Vector3d velocity_vector_, leader_velocity_vector_,
    followers_velocity_vector_;
  arma::colvec data_;
  arma::colvec leader_data_;
  arma::colvec followers_data_;

private:
  OneHotEncoding one_hot_;
};
