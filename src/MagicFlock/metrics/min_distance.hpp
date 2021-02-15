#pragma once

#include <limits>
#include <vector>

#include <MagicFlock/util/logger.hpp>
#include <ignition/math/Vector3.hh>

using namespace ILMR;

template<class QuadrotorType>
class MinDistance
{
public:
  MinDistance()
    : min_global_distance_(std::numeric_limits<double>::max())
    , min_local_distance_(std::numeric_limits<double>::max())
  {
    /* Nothing to do here*/
  }

  double check_global_distance(const std::vector<QuadrotorType>& quads)
  {
    double min_global_distance = std::numeric_limits<double>::max();
    min_global_distance_ = std::numeric_limits<double>::max();
    for (auto&& q : quads) {
      for (std::size_t i = 0; i < q.neighbors().size(); ++i) {
        min_global_distance =
          q.position().Distance(q.neighbors().at(i).position);
        if (min_global_distance < min_global_distance_) {
          min_global_distance_ = min_global_distance;
        }
      }
    }
    return min_global_distance_;
  }
  
  double check_global_distance_no_leader(const std::vector<QuadrotorType>& quads)
  {
    double min_global_distance = std::numeric_limits<double>::max();
    min_global_distance_ = std::numeric_limits<double>::max();
    for (std::size_t j = 1; j < quads.size(); ++j) {
      for (std::size_t i = 1; i < quads.at(j).neighbors().size(); ++i) {
        min_global_distance =
          quads.at(j).position().Distance(quads.at(j).neighbors().at(i).position);
        if (min_global_distance < min_global_distance_) {
          min_global_distance_ = min_global_distance;
        }
      }
    }
    return min_global_distance_;
  }

  double check_local_distance(const QuadrotorType& q)
  {
    double min_local_distance = std::numeric_limits<double>::max();
    min_local_distance_ = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < q.neighbors().size(); ++i) {
      min_local_distance = q.position().Distance(q.neighbors().at(i).position);
      if (min_local_distance < min_local_distance_) {
        min_local_distance_ = min_local_distance;
      }
    }
    return min_local_distance_;
  }

private:
  double min_global_distance_;
  double min_local_distance_;
};
