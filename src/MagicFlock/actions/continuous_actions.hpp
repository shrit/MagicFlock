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
  ignition::math::Vector3d leader_action() const;
  ignition::math::Vector3d& leader_action();
  ignition::math::Vector3d followers_action() const;
  ignition::math::Vector3d& followers_action();
  ContinuousActions to_action(arma::uword index);
  void set_action(arma::colvec data);
  void set_action_leader(arma::colvec data);
  void set_action_followers(arma::colvec data);
  arma::colvec Data();
  arma::colvec leader_data();
  arma::colvec followers_data();
  void calculate_all_possible_actions();

private:
  ignition::math::Vector3d velocity_vector_;
  ignition::math::Vector3d leader_velocity_vector_;
  ignition::math::Vector3d followers_velocity_vector_;
  arma::colvec data_;
  arma::colvec leader_data_;
  arma::colvec followers_data_;
};
