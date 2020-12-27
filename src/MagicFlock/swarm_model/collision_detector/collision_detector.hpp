/**
 * This file implment a basic collision detector
 * It generate a fixed velocity based on the distance between quadrotors
 * If the quadrotors are very close to each other the last velocity
 * vector is inversed to stop any possible collision.
 *
 * author: Omar Shrit <omar@shrit.me>
 *
 */
#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>

#include <limits>
#include <vector>

#include <MagicFlock/util/logger.hpp>

using namespace ILMR;

template<class ActionType>
class CollisionDetector
{
public:
  CollisionDetector();
  CollisionDetector(
    const ignition::math::Vector3d& position,
    const std::vector<ignition::math::Vector3d>& position_of_neighbors,
    ActionType current_action)
    : position_(position)
    , position_of_neighbors_(position_of_neighbors)
    , current_action_(current_action)
    , min_local_distance_(std::numeric_limits<double>::max())
    , velocity_(0, 0, 0)
  {
    // Nothing to do here
  }

  ignition::math::Vector3d Velocity()
  {
    velocity_ = current_action_.action();
    /**
     * Check if quadrotors are close to each other
     * if yes, apply the negative values for the velocity vectors generated
     * here.
     * Otherwise just return the velocity vector
     */
    double min_local_distance;
    min_local_distance_ = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < position_of_neighbors_.size(); ++i) {
      min_local_distance = position_.Distance(position_of_neighbors_.at(i));
      if (min_local_distance < min_local_distance_) {
        min_local_distance_ = min_local_distance;
      }
    }

    if (min_local_distance_ < 0.5) {
      velocity_.X() = -velocity_.X();
      velocity_.Y() = -velocity_.Y();
      velocity_.Z() = -velocity_.Z();
    }
    return velocity_;
  }

  CollisionDetector(CollisionDetector const&) = delete;
  CollisionDetector(CollisionDetector&&) = default;

private:
  ignition::math::Vector3d position_;
  std::vector<ignition::math::Vector3d> position_of_neighbors_;
  ActionType current_action_;
  double min_local_distance_;
  ignition::math::Vector3d velocity_;
};
