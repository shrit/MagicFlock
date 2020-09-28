#pragma once

template<class QuadrotorType>
CheckDestination<QuadrotorType>::CheckDestination(
  const ignition::math::Vector3d& destination)
  : destination_(destination)
{ /* Nothing to do here*/
}

template<class QuadrotorType>
ignition::math::Vector3d&
CheckDestination<QuadrotorType>::destination()
{
  return destination_;
}

template<class QuadrotorType>
ignition::math::Vector3d
CheckDestination<QuadrotorType>::destination() const
{
  return destination_;
}

template<class QuadrotorType>
bool
CheckDestination<QuadrotorType>::has_arrived(
  const std::vector<QuadrotorType>& quads)
{
  bool has_arrived = false;
  ignition::math::Vector3d r_mig{ 0, 0, 0 };

  for (auto&& q : quads) {
    r_mig = destination_ - q.position();
    ILMR::logger::logger_->info("Distance to destination: {}", r_mig);
    if (std::fabs(r_mig.X()) < 1 and std::fabs(r_mig.Y()) < 5) {
      has_arrived = true;
    }
  }

  return has_arrived;
}
