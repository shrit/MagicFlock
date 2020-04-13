#pragma once

#include <vector>
#include <ignition/math6/ignition/math/Vector3.hh>

class Flocking
{

public:
  Flocking(double sepGain,
           double cohGain,
           double migGain,
           double cutoffDist,
           std::vector<ignition::math::Vector3d> position_of_neighbors);

  ignition::math::Vector3d cohesionVelocity();
  ignition::math::Vector3d separationVelocity();
  ignition::math::Vector3d migrationVelocity();
  ignition::math::Vector3d reynoldsVeclocity();
  ignition::math::Vector3d Velocity();

  Flocking(Flocking const&) = delete;
  Flocking(Flocking&&) = default;

private:
  double separation_gain_;
  double cohesion_gain_;
  double migration_gain_;
  double cutoff_distance_;
  int number_of_neighbors_;
  std::vector<ignition::math::Vector3d> position_of_neighbors_;

};
