#pragma once

#include <vector>
#include <limits>

#include <ignition/math/Vector3.hh>
#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

template<class QuadrotorType>
class MaxDistance
{
public:
  MaxDistance() : max_distance_(std::numeric_limits<double>::min())
  { /* Nothing to do here*/
  }

  double check_distance(const std::vector<QuadrotorType>& quads)
  {
    double max_distance;
    for (auto&& q : quads) {
      for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
        max_distance = q.position().Distance(q.neighbor_positions().at(i));
        if (max_distance < max_distance_) {
          max_distance_ = max_distance;
        }
      }
    }
    return max_distance_;
  }

private:
  double max_distance_;
};

