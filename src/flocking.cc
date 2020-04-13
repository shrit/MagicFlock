#include "include/flocking.hh"

Flocking::Flocking(double sepGain,
                   double cohGain,
                   double migGain,
                   double cutoffDist,
                   ignition::math::Vector3d position,
                   std::vector<ignition::math::Vector3d> position_of_neighbors,
                   ignition::math::Vector3d destination_position)
  : separation_gain_(sepGain)
  , cohesion_gain_(cohGain)
  , migration_gain_(migGain)
  , cutoff_distance_(cutoffDist)
  , position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)

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
    total_sum += (position_ - position_of_neighbors_.at(i));
  }
  cohesionVelocity = total_sum * param;
  return cohesionVelocity;
}

ignition::math::Vector3d
Flocking::separationVelocity()
{
  ignition::math::Vector3d separationVelocity;
  double param = -(separation_gain_ / number_of_neighbors_);
  ignition::math::Vector3d total_sum{ 0, 0, 0 };
  ignition::math::Vector3d r_sep{ 0, 0, 0 }, r_sep_norm_2{ 0, 0, 0 };

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    r_sep = (position_ - position_of_neighbors_.at(i));
    double d = r_sep.X() * r_sep.X() + r_sep.Y() * r_sep.Y() +
               r_sep.Z() * r_sep.Z();

    r_sep_norm_2.X() = r_sep.X() / d;
    r_sep_norm_2.Y() = r_sep.Y() / d;
    r_sep_norm_2.Z() = r_sep.Z() / d;

    total_sum += r_sep_norm_2;
  }
  separationVelocity = total_sum * param;
  return separationVelocity;
}

ignition::math::Vector3d
Flocking::migrationVelocity()
{
  ignition::math::Vector3d migrationVelocity;
  ignition::math::Vector3d r_mig;
  r_mig = position_ - destination_position_;
  r_mig.Normalize(); // This will do the entire operation of division
  migrationVelocity = r_mig * migration_gain_;
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
