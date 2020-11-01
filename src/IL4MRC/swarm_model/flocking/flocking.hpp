/*
 * This file implment a modified version of Reynolds flocking model
 * This implementation is described in the following article
 *
 * For more details please see the following paper:
 *
 *  @ARTICLE{8798720,
 *    author={F. {Schilling} and J. {Lecoeur} and F. {Schiano} and D.
 * {Floreano}}, journal={IEEE Robotics and Automation Letters}, title={Learning
 * Vision-Based Flight in Drone Swarms by Imitation}, year={2019}, volume={4},
 *    number={4},
 *    pages={4523-4530},}
 *
 * author: Omar Shrit <omar@shrit.me>
 *
 */

#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <vector>

#include <IL4MRC/util/logger.hpp>

using namespace ILMR;

class Flocking
{

public:
  Flocking(const ignition::math::Vector3d& position,
           const std::vector<ignition::math::Vector3d>& position_of_neighbors,
           const ignition::math::Vector3d& destination_position,
           const ignition::math::Vector3d& max_speed);

  Flocking(const ignition::math::Vector4d& gains,
           const ignition::math::Vector3d& position,
           const std::vector<ignition::math::Vector3d>& position_of_neighbors,
           const ignition::math::Vector3d& destination_position,
           const ignition::math::Vector3d& max_speed);

  ignition::math::Vector3d cohesionVelocity();
  ignition::math::Vector3d separationVelocity();
  ignition::math::Vector3d migrationVelocity();
  ignition::math::Vector3d reynoldsVelocity();
  ignition::math::Vector3d Velocity();
  std::vector<int> OneHotEncodingVelocity();
  Flocking(Flocking const&) = delete;
  Flocking(Flocking&&) = default;

private:
  int number_of_neighbors_;

  /* These default values are reported in this paper:
   * https://ieeexplore.ieee.org/document/8675652
   * The vector values are the following:
   * X = 1  -> cohesion gain
   * Y = 10 -> separation gain
   * Z = 1  -> migration gain
   * W = 100 -> max neighbor distance
   */
  ignition::math::Vector4d gains_{ 1, 10, 1, 100 };
  ignition::math::Vector3d position_;
  std::vector<ignition::math::Vector3d> position_of_neighbors_;
  ignition::math::Vector3d destination_position_;
  ignition::math::Vector3d max_speed_;
};
