#pragma once

template<class QuadrotorType>
CheckDestination<QuadrotorType>::CheckDestination(
  const ignition::math::Vector3d& destination)
  : destination_(destination)
{ /* Nothing to do here*/
}

template<class QuadrotorType>
bool
CheckDestination<QuadrotorType>::has_arrived(
  const std::vector<QuadrotorType>& quads)
{
  bool has_arrived = false;

  for (auto&& q : quads) {
    double r_mig = q.position().Distance(destination_);
    if (r_mig < 1) {
      has_arrived = true;
    }
  }

  return has_arrived;
}
