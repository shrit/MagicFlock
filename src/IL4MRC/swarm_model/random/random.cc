#include "random.hpp"

RandomModel::RandomModel()
  : generator_(random_dev())
  , velocity_(0, 0, 0)
{
  passed_time_ = 0;
}

ignition::math::Vector3d
RandomModel::Velocity(ignition::math::Vector4d axis_speed, double passed_time)
{
  axis_ = axis_speed.W();

  std::uniform_real_distribution<> distribution_real_x_(-axis_speed.X(),
                                                        axis_speed.X());
  std::uniform_real_distribution<> distribution_real_y_(-axis_speed.Y(),
                                                        axis_speed.Y());
  std::uniform_real_distribution<> distribution_real_z_(-axis_speed.Z(),
                                                        axis_speed.Z());
  logger::logger_->info("Elapsed time {}", passed_time);
  if (passed_time > passed_time_ + 5) {
    logger::logger_->info("Generate random velocity");
    passed_time_ = passed_time;
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
  }
  return velocity_;
}
