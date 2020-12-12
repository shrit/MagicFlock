#include "flocking.hpp"

Flocking::Flocking(const ignition::math::Vector3d& position,
                   const std::vector<neighbor>& neighbors,
                   const ignition::math::Vector3d& destination_position,
                   const ignition::math::Vector3d& max_speed)
  : position_(position)
  , number_of_neighbors_(neighbors.size())
  , destination_position_(destination_position)
  , max_speed_(max_speed)
{
  // The maximum number of neighbor should be total (size - 1).
  // We have reference to our self
  for (std::size_t i = 0; i < neighbors.size(); ++i) {
    position_of_neighbors_.push_back(neighbors.at(i).position);
  }
}

Flocking::Flocking(const ignition::math::Vector4d& gains,
                   const ignition::math::Vector3d& position,
                   const std::vector<neighbor>& neighbors,
                   const ignition::math::Vector3d& destination_position,
                   const ignition::math::Vector3d& max_speed,
                   const bool& leader)
  : gains_(gains)
  , position_(position)
  , number_of_neighbors_(neighbors.size())
  , destination_position_(destination_position)
  , max_speed_(max_speed)
{
  // The maximum number of neighbor should be total (size - 1).
  // We have reference to our self
  for (std::size_t i = 0; i < neighbors.size(); ++i) {
    position_of_neighbors_.push_back(neighbors.at(i).position);
  }

  // Leader is always the first quadrotors id = 0
  if (leader) {
    number_of_neighbors_--;
    position_of_neighbors_.erase(position_of_neighbors_.begin());
  }
  logger::logger_->debug("My postion: {}\n", position_);
  for (auto i : position_of_neighbors_) {
    logger::logger_->debug("neighbor: {}\n", i);
  }
}

ignition::math::Vector3d
Flocking::cohesionVelocity()
{
  double param = gains_.X() / number_of_neighbors_;
  ignition::math::Vector3d total_sum{ 0, 0, 0 }, r_cohs{ 0, 0, 0 };

  for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
    r_cohs = position_of_neighbors_.at(i) - position_;
    total_sum += r_cohs;
  }
  logger::logger_->debug("cohesion total sum: {}\n", total_sum);
  logger::logger_->debug("param cohesion: {}\n", param);
  cohesionVelocity_ = total_sum * param;
  return cohesionVelocity_;
}

ignition::math::Vector3d
Flocking::separationVelocity()
{
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
  logger::logger_->debug("Separation total sum: {}", total_sum);
  logger::logger_->debug("param: {}\n", param);
  separationVelocity_ = total_sum * param;
  logger::logger_->debug("SeparationVelocity: {}\n", separationVelocity_);
  return separationVelocity_;
}

ignition::math::Vector3d
Flocking::migrationVelocity()
{
  ignition::math::Vector3d r_mig;
  r_mig = destination_position_ - position_;
  r_mig = r_mig.Normalize(); // This will do the entire operation of division
  migrationVelocity_ = r_mig * gains_.Z();
  logger::logger_->debug("Gains Z: {}", gains_.Z());
  logger::logger_->debug("migration {}", migrationVelocity_);
  return migrationVelocity_;
}

ignition::math::Vector3d
Flocking::reynoldsVelocity()
{
  ignition::math::Vector3d coh, sep, rey;
  coh = cohesionVelocity();
  sep = separationVelocity();
  rey = coh + sep;
  return rey;
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
  logger::logger_->debug("Cohesion Velocity: {}\n", coh);
  logger::logger_->debug("Final velocity: {}\n", total);

  /* Setup the max speed on each axis instead of the generated speed */
  if (std::fabs(total.X()) > max_speed_.X()) {
    if (total.X() < 0) {
      total.X(-max_speed_.X());
    } else {
      total.X(max_speed_.X());
    }
  } else if (std::fabs(total.Y()) > max_speed_.Y()) {
    if (total.Y() < 0) {
      total.Y(-max_speed_.Y());
    } else {
      total.Y(max_speed_.Y());
    }
  } else if (std::fabs(total.Z()) > max_speed_.Z()) {
    if (total.Z() < 0) {
      total.Z(-max_speed_.Z());
    } else {
      total.Z(max_speed_.Z());
    }
  }
  return total;
}

std::vector<int>
Flocking::OneHotEncodingVelocity()
{
  std::vector<int> one_hot{
    0, // mig +x
    0, // mig -x
    0, // mig +y
    0, // mig -y
    0, // mig +z
    0, // mig -z
    0, // rule +x
    0, // rule -x
    0, // rule +y
    0, // rule -y
    0, // rule +z
    0  // rule -z
  };
  ignition::math::Vector3d coh, sep, mig, rule, abs_mig, abs_rule;
  coh = cohesionVelocity();
  sep = separationVelocity();
  mig = migrationVelocity();

  rule = coh + sep;
  abs_rule = rule.Abs();
  abs_mig = mig.Abs();
  double max_rule = abs_rule.Max();
  double max_mig = abs_mig.Max();
  if (max_rule < max_mig) {
    if (mig.Max() > std::fabs(mig.Min())) {
      if (mig.Max() == mig.X())
        one_hot.at(0) = 1;
      else if (mig.Max() == mig.Y())
        one_hot.at(2) = 1;
      else if (mig.Max() == mig.Z())
        one_hot.at(4) = 1;
    } else if (mig.Max() < std::fabs(mig.Min())) {
      if (mig.Min() == mig.X())
        one_hot.at(1) = 1;
      else if (mig.Min() == mig.Y())
        one_hot.at(3) = 1;
      else if (mig.Min() == mig.Z())
        one_hot.at(5) = 1;
    }
  } else {
    if (rule.Max() > std::fabs(rule.Min())) {
      if (rule.Max() == rule.X())
        one_hot.at(6) = 1;
      else if (rule.Max() == rule.Y())
        one_hot.at(8) = 1;
      else if (rule.Max() == rule.Z())
        one_hot.at(10) = 1;
    } else if (rule.Max() < std::fabs(rule.Min())) {
      if (rule.Min() == rule.X())
        one_hot.at(7) = 1;
      else if (rule.Min() == rule.Y())
        one_hot.at(9) = 1;
      else if (rule.Min() == rule.Z())
        one_hot.at(11) = 1;
    }
  }
  return one_hot;
}
