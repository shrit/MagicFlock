#include "include/olfati_saber.hh"

OlfatiSaber::OlfatiSaber(
  const ignition::math::Vector3d& position,
  const std::vector<ignition::math::Vector3d>& position_of_neighbors,
  const ignition::math::Vector3d& destination_position)
  : position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)

{
  // the size of neighbor should be total size-1.
  // We have reference to our self
  number_of_neighbors_ = position_of_neighbors.size();
}

OlfatiSaber::OlfatiSaber(
  const ignition::math::Vector4d& gains,
  const ignition::math::Vector3d& position,
  const std::vector<ignition::math::Vector3d>& position_of_neighbors,
  const ignition::math::Vector3d& destination_position)
  : gains_(gains)
  , position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)

{
  // the size of neighbor should be total size-1.
  // We have reference to our self
  number_of_neighbors_ = position_of_neighbors.size();
  logger::logger_->debug("My postion: {}\n", position_);
  for (auto i : position_of_neighbors_) {
    logger::logger_->debug("neighbor: {}\n", i);
  }
}

ignition::math::Vector3d
OlfatiSaber::cohesionVelocity()
{
  ignition::math::Vector3d cohesionVelocity{ 0, 0, 0 };

  return cohesionVelocity;
}

ignition::math::Vector3d
OlfatiSaber::separationVelocity()
{
  ignition::math::Vector3d separationVelocity{ 0, 0, 0 };

  return separationVelocity;
}

ignition::math::Vector3d
OlfatiSaber::migrationVelocity()
{
  ignition::math::Vector3d migrationVelocity{ 0, 0, 0 };

  return migrationVelocity;
}

ignition::math::Vector3d
OlfatiSaber::Velocity()
{
  ignition::math::Vector3d coh, sep, mig, total;
  coh = cohesionVelocity();
  sep = separationVelocity();
  mig = migrationVelocity();
  total = coh + sep + mig;
  logger::logger_->debug("Migration Velocity: {}\n", mig);
  logger::logger_->debug("Separation velocity: {}\n", sep);
  logger::logger_->debug("cohesionVelocity: {}\n", coh);
  logger::logger_->debug("Final velocity: {}\n", total);
  return total;
}

