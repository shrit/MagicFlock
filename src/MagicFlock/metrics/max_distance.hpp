#pragma once

#include <limits>
#include <vector>

#include <MagicFlock/util/logger.hpp>
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
    double max_global_distance = std::numeric_limits<double>::min();
    max_global_distance_ = std::numeric_limits<double>::min();
    for (auto&& q : quads) {
      for (std::size_t i = 0; i < q.neighbors().size(); ++i) {
        max_global_distance =
          q.position().Distance(q.neighbors().at(i).position);
        if (max_global_distance > max_global_distance_) {
          max_global_distance_ = max_global_distance;
        }
      }
    }
    return max_global_distance_;
  }
  
  double check_global_distance_no_leader(const std::vector<QuadrotorType>& quads)
  {
    double max_global_distance = std::numeric_limits<double>::min();
    max_global_distance_ = std::numeric_limits<double>::min();
    for (std::size_t j = 1; j < quads.size(); ++j) {
      for (std::size_t i = 1; i < quads.at(j).neighbors().size(); ++i) {
        max_global_distance =
          quads.at(j).position().Distance(quads.at(j).neighbors().at(i).position);
        if (max_global_distance > max_global_distance_) {
          max_global_distance_ = max_global_distance;
        }
      }
    }
    return max_global_distance_;
  }

  double check_local_distance(const QuadrotorType& q)
  {
    double max_local_distance = std::numeric_limits<double>::min();
    max_local_distance_ = std::numeric_limits<double>::min();
    for (std::size_t i = 0; i < q.neighbors().size(); ++i) {
      max_local_distance = q.position().Distance(q.neighbors().at(i).position);
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
