/*
 * This file implment a modified version of Reynolds flocking model
 * This implementation is described in the following article
 *
 * For more details please see the following paper:
 *
 *
 * author: Omar Shrit <omar@shrit.me>
 *
 */

#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <vector>

#include "logger.hh"

using namespace ILMR;

class OlfatiSaber
{

public:
  OlfatiSaber(
    const ignition::math::Vector3d& position,
    const std::vector<ignition::math::Vector3d>& position_of_neighbors,
    const ignition::math::Vector3d& destination_position);

  OlfatiSaber(
    const ignition::math::Vector4d& gains,
    const ignition::math::Vector3d& position,
    const std::vector<ignition::math::Vector3d>& position_of_neighbors,
    const ignition::math::Vector3d& destination_position);

  ignition::math::Vector3d cohesionVelocity();
  ignition::math::Vector3d separationVelocity();
  ignition::math::Vector3d migrationVelocity();
  ignition::math::Vector3d Velocity();

  OlfatiSaber(OlfatiSaber const&) = delete;
  OlfatiSaber(OlfatiSaber&&) = default;

private:
  int number_of_neighbors_;

  ignition::math::Vector3d position_;
  std::vector<ignition::math::Vector3d> position_of_neighbors_;
  ignition::math::Vector3d destination_position_;
};
