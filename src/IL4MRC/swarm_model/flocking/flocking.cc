#include "flocking.hpp"

Flocking::Flocking(
  const ignition::math::Vector3d& position,
  const std::vector<ignition::math::Vector3d>& position_of_neighbors,
  const ignition::math::Vector3d& destination_position,
  const ignition::math::Vector3d& max_speed)
  : position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)
  , max_speed_(max_speed)
{
  // The maximum number of neighbor should be total (size - 1).
  // We have reference to our self
  number_of_neighbors_ = position_of_neighbors.size();
}

Flocking::Flocking(
  const ignition::math::Vector4d& gains,
  const ignition::math::Vector3d& position,
  const std::vector<ignition::math::Vector3d>& position_of_neighbors,
  const ignition::math::Vector3d& destination_position,
  const ignition::math::Vector3d& max_speed)
  : gains_(gains)
  , position_(position)
  , position_of_neighbors_(position_of_neighbors)
  , destination_position_(destination_position)
  , max_speed_(max_speed)
{
  // The maximum number of neighbor should be total (size - 1).
  // We have reference to our self
  number_of_neighbors_ = position_of_neighbors.size();
  logger::logger_->debug("My postion: {}\n", position_);
  for (auto i : position_of_neighbors_) {
    logger::logger_->debug("neighbor: {}\n", i);
  }
}

ignition::math::Vector3d
Flocking::cohesionVelocity()
{
  ignition::math::Vector3d cohesionVelocity{ 0, 0, 0 };
  double param = gains_.X() / number_of_neighbors_;
  ignition::math::Vector3d total_sum{ 0, 0, 0 }, r_cohs{ 0, 0, 0 };

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    r_cohs = position_of_neighbors_.at(i) - position_;
    total_sum += r_cohs;
  }
  logger::logger_->debug(
    "cohesion total sum: {}", total_sum, " param: {}\n", param);
  cohesionVelocity = total_sum * param;
  return cohesionVelocity;
}

ignition::math::Vector3d
Flocking::separationVelocity()
{
  ignition::math::Vector3d separationVelocity{ 0, 0, 0 };
  double param = -(gains_.Y() / number_of_neighbors_);
  ignition::math::Vector3d total_sum{ 0, 0, 0 };
  ignition::math::Vector3d r_sep{ 0, 0, 0 }, r_sep_norm_2{ 0, 0, 0 };

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    r_sep = position_of_neighbors_.at(i) - position_;
    double d = r_sep.SquaredLength();

    r_sep_norm_2.X() = r_sep.X() / d;
    r_sep_norm_2.Y() = r_sep.Y() / d;
    r_sep_norm_2.Z() = r_sep.Z() / d;

    total_sum += r_sep_norm_2;
  }
  logger::logger_->debug(
    "Separation total sum: ", total_sum, " param: {}\n", param);
  separationVelocity = total_sum * param;
  return separationVelocity;
}

ignition::math::Vector3d
Flocking::migrationVelocity()
{
  ignition::math::Vector3d migrationVelocity{ 0, 0, 0 };
  ignition::math::Vector3d r_mig;
  r_mig = destination_position_ - position_;
  r_mig = r_mig.Normalize(); // This will do the entire operation of division
  migrationVelocity = r_mig * gains_.Z();
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
  ignition::math::Vector3d coh, sep, mig, total;
  coh = cohesionVelocity();
  sep = separationVelocity();
  mig = migrationVelocity();
  total = coh + sep + mig;
  logger::logger_->debug("Migration Velocity: {}\n", mig);
  logger::logger_->debug("Separation velocity: {}\n", sep);
  logger::logger_->debug("cohesionVelocity: {}\n", coh);
  logger::logger_->debug("Final velocity: {}\n", total);

  /* Setup the max speed on each axis instead of the generated speed */
  if (std::fabs(total.X()) > max_speed_.X()) {
    if (total.X() < 0){
      total.X(-max_speed_.X());
    } else {
      total.X(max_speed_.X());
    }
  } else if (std::fabs(total.Y()) > max_speed_.Y()) {
    if (total.Y() < 0){
      total.Y(-max_speed_.Y());
    } else {
      total.Y(max_speed_.Y());
    }
  } else if (std::fabs(total.Z()) > max_speed_.Z()) {
    if (total.Z() < 0){
      total.Z(-max_speed_.Z());
    } else {
      total.Z(max_speed_.Z());
    }
  }
  return total;
}
