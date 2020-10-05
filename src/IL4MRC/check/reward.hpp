#pragma once

#include <ignition/math/Vector3.hh>
#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

template<class QuadrotorType>
class Reward
{
public:
  Reward()
  {
    /* Nothing to do here */
  }

  double Calculate_reward(const QuadrotorType& quad)
  {
    double reward = 0.0;
    for (std::size_t i = 0; i < quad.neighbor_positions().size(); ++i) {
      double distance = quad.position().Distance(quad.neighbor_positions().at(i));
      ILMR::logger::logger_->debug("Distance to neigh: {}", distance);
      if (distance > 1.10 and distance < 2.5) {
        reward = 1;
      } else if (distance < 1.10) {
        reward = - 30;
      } else if (distance > 2.5 and distance < 6) {
        reward = 0; 
      } else {
        reward = -10;
      }
      return reward;
    }
  }
  
};
