#pragma once

#include <limits>
#include <vector>

#include <IL4MRC/util/logger.hpp>
#include <ignition/math/Vector3.hh>

using namespace ILMR;

template<class QuadrotorType>
class MaxDistance
{
public:
  MaxDistance()
    : max_global_distance_(std::numeric_limits<double>::min())
    , max_local_distance_(std::numeric_limits<double>::min())
  {
    /* Nothing to do here*/
  }

  double check_global_distance(const std::vector<QuadrotorType>& quads)
  {
    double max_global_distance;
    for (auto&& q : quads) {
      for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
        max_global_distance =
          q.position().Distance(q.neighbor_positions().at(i));
        if (max_global_distance > max_global_distance_) {
          max_global_distance_ = max_global_distance;
        }
      }
    }
    return max_global_distance_;
  }

  double check_local_distance(const QuadrotorType& q)
  {
    double max_local_distance;
    for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
      max_local_distance = q.position().Distance(q.neighbor_positions().at(i));
      if (max_local_distance > max_local_distance_) {
        max_local_distance_ = max_local_distance;
      }
    }
    return max_local_distance_;
  }

private:
  double max_global_distance_;
  double max_local_distance_;
};
