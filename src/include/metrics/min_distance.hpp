#pragma once

#include <limits>

#include <ignition/math/Vector3.hh>
#include "logger.hpp"

using namespace ILMR;

template<class QuadrotorType>
class MinDistance
{
public:
  MinDistance() : min_distance_(std::numeric_limits<double>::max())
  { /* Nothing to do here*/
  }

  void check_distance(const std::vector<QuadrotorType>& quads);
  double min_distance();

private:
  double min_distance_;
};

#include "min_distance_impl.hpp"

