/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotors as a velocity_vector
 * This class is a wrapper for actions to be able to create state.
 *
 * ContinuousActions means that the action are represented as the velocity
 * vector on x, y, z. The value on each one of (x,y,z) determins the direction
 * for which the quadrotor is moving.
 *
 */
#pragma once
#include <vector>

#include <ignition/math6/ignition/math/Vector3.hh>
#include <mlpack/prereqs.hpp>

class ContinuousActions
{
public:
  ContinuousActions();
  ignition::math::Vector3d action() const;
  ignition::math::Vector3d& action();
  std::vector<int> one_hot_action() const;
  std::vector<int>& one_hot_action();
  ContinuousActions int_to_action(arma::uword index);
  void set_action(arma::colvec data);
  arma::colvec Data();
  arma::mat all_possible_actions();
  arma::mat all_possible_actions_one_hot();

private:
  ignition::math::Vector3d velocity_vector_;
  std::vector<int> one_hot_encoding_action_{
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };
  arma::colvec data_;
};
