#pragma once

/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
 */

#include <memory>

/*  Local includes */
#include "compute_distance.hh"
#include "gaussian_noise.hh"
#include "math_tools.hh"

template<class simulator_t, class NoiseType>
class State
{

public:
  State();

  State(const arma::colvec& data);

  State(std::shared_ptr<simulator_t> sim_interface,
        unsigned int id,
        std::vector<unsigned int> nearest_neighbors,
        NoiseType noise);

  State(std::shared_ptr<simulator_t> sim_interface,
        unsigned int id,
        std::vector<unsigned int> nearest_neighbors);

  arma::colvec Data() const;

  arma::colvec& Data();

  double AltitudeDiff() const;

  double& AltitudeDiff();

  arma::colvec Distances() const;

  arma::colvec& Distances();

  std::map<unsigned int, double> neighbor_dists_3D() const;

  const arma::colvec& Encode() const;

  static constexpr size_t dimension = 3;

private:
  int id_;
  ComputeDistance dist_;
  Math_tools mtools_;
  std::map<unsigned int, double> neighbor_dists_3D_;

  arma::colvec data_;
  /*  Create a shared pointer to a simulator interface The interface
      need to have all the required data about the quadcopter*/
  std::shared_ptr<simulator_t> sim_interface_;
  std::vector<ignition::math::Vector3d>::iterator leader_;
  std::vector<ignition::math::Vector3d>::iterator follower_;
};

#include "state.hxx"
