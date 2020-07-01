#pragma once

#include <ignition/math/Vector3.hh>

using namespace ILMR;

template<class QuadrotorType>
class CheckDestination
{
public:
  CheckDestination()
  { /* Nothing to do here*/
  }
  CheckDestination(const ignition::math::Vector3d& destination);

  bool has_arrived(const std::vector<QuadrotorType>& quads);

private:
  ignition::math::Vector3d destination_;
};

#include "check_destination.hxx"
