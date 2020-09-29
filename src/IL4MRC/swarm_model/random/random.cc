#include "random.hpp"

RandomModel::RandomModel(ignition::math::Vector4d axis_speed)
  : axis_(axis_speed.W())
  , distribution_real_x_(-axis_speed.X(), axis_speed.X())
  , distribution_real_y_(-axis_speed.Y(), axis_speed.Y())
  , distribution_real_z_(-axis_speed.Z(), axis_speed.Z())

, generator_(random_dev())
  , velocity_(0, 0, 0)
{
  // Nothing to do here
}

ignition::math::Vector3d
RandomModel::Velocity()
{
  if (axis_ == 0) {
    // generate action on x axis
    velocity_.X() = distribution_real_x_(generator_);

  } else if (axis_ == 1) {
    // generate action on y axis
    velocity_.Y() = distribution_real_y_(generator_);

  } else if (axis_ == 2) {
    // generate action on z axis
    velocity_.Z() = distribution_real_z_(generator_);

  } else if (axis_ == 3) {
    // generate actions on x,y axis
    velocity_.X() = distribution_real_x_(generator_);
    velocity_.Y() = distribution_real_y_(generator_);

  } else if (axis_ == 4) {
    // generate actions on x,y,z axis
    velocity_.X() = distribution_real_x_(generator_);
    velocity_.Y() = distribution_real_y_(generator_);
    velocity_.Z() = distribution_real_z_(generator_);
  }
  return velocity_;  
}
