/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotors as a velocity_vector
 * This class is a wrapper for actions to be able to create state, action
 *
 */
#pragma once

#include <ignition/math6/ignition/math/Vector3.hh>
#include <mlpack/prereqs.hpp>

class ContinuousActions
{
public:
  ContinuousActions();
  ignition::math::Vector3d action() const;
  ignition::math::Vector3d& action();
  void set_action(arma::colvec data);
  arma::colvec Data();

private:
  ignition::math::Vector3d velocity_vector_;
  arma::colvec data_;
};
