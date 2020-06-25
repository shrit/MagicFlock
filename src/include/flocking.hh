#pragma once

#include <ignition/math6/ignition/math/Vector3.hh>
#include <ignition/math6/ignition/math/Vector4.hh>
#include <vector>

#include "logger.hh"

using namespace ILMR;

class Flocking
{

public:
  Flocking(const ignition::math::Vector3d& position,
           const std::vector<ignition::math::Vector3d>& position_of_neighbors,
           const ignition::math::Vector3d& destination_position);

  Flocking(const ignition::math::Vector4d& gains,
           const ignition::math::Vector3d& position,
           const std::vector<ignition::math::Vector3d>& position_of_neighbors,
           const ignition::math::Vector3d& destination_position);

  ignition::math::Vector3d cohesionVelocity();
  ignition::math::Vector3d separationVelocity();
  ignition::math::Vector3d migrationVelocity();
  ignition::math::Vector3d reynoldsVelocity();
  ignition::math::Vector3d Velocity();

  Flocking(Flocking const&) = delete;
  Flocking(Flocking&&) = default;

private:
  int number_of_neighbors_;

  // These default values are reported in this paper:
  // https://ieeexplore.ieee.org/document/8675652
  ignition::math::Vector4d gains_{ 1, 1, 10, 100 };
  ignition::math::Vector3d position_;
  std::vector<ignition::math::Vector3d> position_of_neighbors_;
  ignition::math::Vector3d destination_position_;
};
