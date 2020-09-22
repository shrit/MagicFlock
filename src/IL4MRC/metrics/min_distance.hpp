#pragma once

#include <limits>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

template<class QuadrotorType>
class MinDistance
{
public:
  MinDistance() : min_distance_(std::numeric_limits<double>::max())
  { 
    /* Nothing to do here*/
  }

  double check_distance(const std::vector<QuadrotorType>& quads)
  {
    double min_distance;
    for (auto&& q : quads) {
      for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
        min_distance = q.position().Distance(q.neighbor_positions().at(i));
        if (min_distance < min_distance_) {
          min_distance_ = min_distance;
        }
      }
    }
    return min_distance_;
  }

private:
  double min_distance_;
};

#include "min_distance_impl.hpp"

