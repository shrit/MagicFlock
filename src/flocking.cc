#include "include/flocking.hh"

Flocking::Flocking(ignition::math::Vector3d position,
                   std::vector<ignition::math::Vector3d> position_of_neighbors,
                   ignition::math::Vector3d destination_position)
  : position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)

{
  // the size of neighbor should be total size-1.
  // We have reference to our self
  number_of_neighbors_ = position_of_neighbors.size();
}

Flocking::Flocking(ignition::math::Vector4d gains,
                   ignition::math::Vector3d position,
                   std::vector<ignition::math::Vector3d> position_of_neighbors,
                   ignition::math::Vector3d destination_position)
  : gains_(gains)
  , position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)

{
  // the size of neighbor should be total size-1.
  // We have reference to our self
  number_of_neighbors_ = position_of_neighbors.size();
}

ignition::math::Vector3d
Flocking::cohesionVelocity()
{
  ignition::math::Vector3d cohesionVelocity{ 0, 0, 0 };
  double param = gains_.X() / number_of_neighbors_;
  ignition::math::Vector3d total_sum{ 0, 0, 0 };

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    total_sum += (position_ - position_of_neighbors_.at(i));
  }
  std::cout << "cohesion total sum" << total_sum << " param: " << param
            << std::endl;
  cohesionVelocity = total_sum * param;
  std::cout << "cohesionVelocity: " << cohesionVelocity << std::endl;
  return cohesionVelocity;
}

ignition::math::Vector3d
Flocking::separationVelocity()
{
  ignition::math::Vector3d separationVelocity;
  double param = -(gains_.Y() / number_of_neighbors_);
  ignition::math::Vector3d total_sum{ 0, 0, 0 };
  ignition::math::Vector3d r_sep{ 0, 0, 0 }, r_sep_norm_2{ 0, 0, 0 };
  std::cout << "separation_gain_" << gains_.Y() << std::endl;

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    r_sep = (position_ - position_of_neighbors_.at(i));
    double d =
      r_sep.X() * r_sep.X() + r_sep.Y() * r_sep.Y() + r_sep.Z() * r_sep.Z();

    r_sep_norm_2.X() = r_sep.X() / d;
    r_sep_norm_2.Y() = r_sep.Y() / d;
    r_sep_norm_2.Z() = r_sep.Z() / d;

    total_sum += r_sep_norm_2;
  }
  std::cout << "Separation total sum: " << total_sum << " param: " << param
            << std::endl;
  separationVelocity = total_sum * param;
  std::cout << "Separation velocity: " << separationVelocity << std::endl;
  return separationVelocity;
}

ignition::math::Vector3d
Flocking::migrationVelocity()
{
  ignition::math::Vector3d migrationVelocity;
  ignition::math::Vector3d r_mig;
  r_mig = position_ - destination_position_;
  r_mig.Normalize(); // This will do the entire operation of division
  migrationVelocity = r_mig * gains_.Z();
  std::cout << "Migration Velocity: " << migrationVelocity << std::endl;
  return migrationVelocity;
}

ignition::math::Vector3d
Flocking::reynoldsVelocity()
{
  return cohesionVelocity() + migrationVelocity();
}

ignition::math::Vector3d
Flocking::Velocity()
{
  std::cout << "Final velocity:"
            << cohesionVelocity() + separationVelocity() + migrationVelocity()
            << std::endl;
  return cohesionVelocity() + separationVelocity() + migrationVelocity();
}
