#include "random.hpp"

RandomModel::RandomModel(ignition::math::Vector3d axis_speed)
  : axis_(axis_speed.X())
  , distribution_real_(axis_speed.Y(), axis_speed.Z())
  , generator_(random_dev())
{
  // Nothing to do here
}

ignition::math::Vector3d
RandomModel::Velocity()
{
  ignition::math::Vector3d velocity{ 0, 0, 0 };

  if (axis_ == 0) {
    // generate action on x axis
    velocity.X() = distribution_real_(generator_);

  } else if (axis_ == 1) {
    // generate action on y axis
    velocity.Y() = distribution_real_(generator_);

  } else if (axis_ == 2) {
    // generate action on z axis
    velocity.Z() = distribution_real_(generator_);

  } else if (axis_ == 3) {
    // generate actions on x,y axis
    velocity.X() = distribution_real_(generator_);
    velocity.Y() = distribution_real_(generator_);

  } else if (axis_ == 4) {
    // generate actions on x,y,z axis
    velocity.X() = distribution_real_(generator_);
    velocity.Y() = distribution_real_(generator_);
    velocity.Z() = distribution_real_(generator_);
  }

  return velocity;
}
