#include "include/flocking.hh"

Flocking::Flocking(double sepGain,
                   double cohGain,
                   double migGain,
                   double cutoffDist,
                   std::vector<ignition::math::Vector3d> position_of_neighbors)
  : separation_gain_(sepGain)
  , cohesion_gain_(cohGain)
  , migration_gain_(migGain)
  , cutoff_distance_(cutoffDist)

{
  number_of_neighbors_ = position_of_neighbors.size();
}

ignition::math::Vector3d
Flocking::cohesionVelocity()
{
  ignition::math::Vector3d cohesionVelocity;
  double param = cohesion_gain_ / number_of_neighbors_;
  ignition::math::Vector3d total_sum{ 0, 0, 0 };

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    total_sum += position_of_neighbors_.at(i);
 
  }
  cohesionVelocity = total_sum * param;
  return cohesionVelocity;
}

ignition::math::Vector3d
Flocking::separationVelocity()
{
  ignition::math::Vector3d separationVelocity;

  return separationVelocity;
}

ignition::math::Vector3d
Flocking::migrationVelocity()
{  
  ignition::math::Vector3d migrationVelocity;

  return migrationVelocity;

}

ignition::math::Vector3d
Flocking::reynoldsVeclocity()
{
  return cohesionVelocity() + migrationVelocity();
}

ignition::math::Vector3d
Flocking::Velocity()
{
  return cohesionVelocity() + separationVelocity() + migrationVelocity();
}
