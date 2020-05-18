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

template<class NoiseType, class ContainerType>
class State
{

public:
  State();

  State(const arma::colvec& data);

  State(unsigned int id,
        std::vector<unsigned int> nearest_neighbors,
        NoiseType noise);

  State(unsigned int id,
        std::vector<unsigned int> nearest_neighbors);

  arma::colvec Data() const;

  arma::colvec& Data();

  arma::colvec RSSI() const;

  arma::colvec& RSSI();

  arma::colvec TOAs() const;
  
  arma::colvec& TOAs();

  std::map<unsigned int, double> neighbor_dists_3D() const;

  const arma::colvec& Encode() const;

  static constexpr size_t dimension = 3;

private:
  int id_;
  ComputeDistance dist_;
  Math_tools mtools_;
  std::map<unsigned int, double> neighbor_dists_3D_;

  arma::colvec data_;
};

#include "state.hxx"
