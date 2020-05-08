#pragma once

#include <ignition/math6/ignition/math/Vector3.hh>
#include <vector>

class Flocking
{

public:
  Flocking(double sepGain,
           double cohGain,
           double migGain,
           double cutoffDist,
           ignition::math::Vector3d position,
           std::vector<ignition::math::Vector3d> position_of_neighbors,
           ignition::math::Vector3d destination_position);

  ignition::math::Vector3d cohesionVelocity();
  ignition::math::Vector3d separationVelocity();
  ignition::math::Vector3d migrationVelocity();
  ignition::math::Vector3d reynoldsVelocity();
  ignition::math::Vector3d Velocity();

  Flocking(Flocking const&) = delete;
  Flocking(Flocking&&) = default;

private:
  double separation_gain_;
  double cohesion_gain_;
  double migration_gain_;
  double cutoff_distance_;
  int number_of_neighbors_;
  ignition::math::Vector3d position_;
  std::vector<ignition::math::Vector3d> position_of_neighbors_;
  ignition::math::Vector3d destination_position_;
};
