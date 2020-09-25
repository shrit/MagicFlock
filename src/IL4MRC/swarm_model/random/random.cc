#include "random.hpp"

RandomModel::RandomModel(double max_speed, double min_speed, int axis)
  : axis_(axis)
  , distribution_real_(min_speed, max_speed)
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
    // generate actions on x,y axis

    velocity.X() = distribution_real_(generator_);
    velocity.Y() = distribution_real_(generator_);

  } else if (axis_ == 2) {
    // generate actions on x,y,z axis
    velocity.X() = distribution_real_(generator_);
    velocity.Y() = distribution_real_(generator_);
    velocity.Z() = distribution_real_(generator_);
  }

  return velocity;
}
