#include "random.hpp"

RandomModel::RandomModel(ignition::math::Vector3d axis_speed)
  : axis_(axis_speed.X())
  , distribution_real_(axis_speed.Y(), axis_speed.Z())
  , generator_(random_dev())
  , velocity_(0, 0, 0)
{
  // Nothing to do here
}

ignition::math::Vector3d
RandomModel::Velocity() const
{
  if (axis_ == 0) {
    // generate action on x axis
    velocity_.X() = distribution_real_(generator_);

  } else if (axis_ == 1) {
    // generate action on y axis
    velocity_.Y() = distribution_real_(generator_);

  } else if (axis_ == 2) {
    // generate action on z axis
    velocity_.Z() = distribution_real_(generator_);

  } else if (axis_ == 3) {
    // generate actions on x,y axis
    velocity_.X() = distribution_real_(generator_);
    velocity_.Y() = distribution_real_(generator_);

  } else if (axis_ == 4) {
    // generate actions on x,y,z axis
    velocity_.X() = distribution_real_(generator_);
    velocity_.Y() = distribution_real_(generator_);
    velocity_.Z() = distribution_real_(generator_);
  }
  return velocity_;  
}
