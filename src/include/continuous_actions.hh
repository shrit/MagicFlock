/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotors as a velocity_vector
 * This class is a wrapper for actions to be able to create state, action
 *
 */
#pragma once

#include <ignition/math6/ignition/math/Vector3.hh>

class ContinuousActions
{
public:
  ContinuousActions();
  ignition::math::Vector3d Data() const;
  ignition::math::Vector3d& Data();

private:
  ignition::math::Vector3d velocity_vector_;
};
