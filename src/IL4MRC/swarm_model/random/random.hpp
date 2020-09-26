/**
 * This file implment a basic random model
 * It generate a fixed velocity based on a random
 * The velocity vector is fixed since the start of the
 * generation and then it changes next time you start this model
 *
 *
 * author: Omar Shrit <omar@shrit.me>
 *
 */
#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>

#include <random>
#include <vector>

#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

class RandomModel
{

public:
  RandomModel(ignition::math::Vector3d axis_speed);
  ignition::math::Vector3d Velocity();

  RandomModel(RandomModel const&) = delete;
  RandomModel(RandomModel&&) = default;

private:
  int axis_;
  std::uniform_real_distribution<> distribution_real_;
  std::random_device random_dev;
  std::mt19937 generator_;
};
