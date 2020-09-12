#pragma once

#include <limits>

#include <ignition/math/Vector3.hh>
#include "logger.hh"

using namespace ILMR;

template<class QuadrotorType>
class MaxDistance
{
public:
  MaxDistance() : max_distance_(std::numeric_limits<double>::min())
  { /* Nothing to do here*/
  }

  void check_distance(const std::vector<QuadrotorType>& quads);
  double max_distance();

private:
  double max_distance_;
};

