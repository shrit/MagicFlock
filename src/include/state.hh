#pragma once

/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
 */

/*  Local includes */
#include "global.hh"
#include "math_tools.hh"

namespace lt = local_types;
class GaussianNoise;

template<class simulator_t, class NoiseType>
class State
{

public:
  State();

  State(const arma::colvec& data);

  template<
    typename std::enable_if<std::is_same<NoiseType, GaussianNoise>::value,
                            bool>::type>
  State(std::shared_ptr<simulator_t> sim_interface,
        unsigned int id,
        std::vector<unsigned int> nearest_neighbors,
        NoiseType apply_noise);

  template<
    typename std::enable_if<!std::is_same<NoiseType, GaussianNoise>::value,
                            bool>::type>
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
  Math_tools mtools_;
  std::map<unsigned int, double> neighbor_dists_3D_;

  arma::colvec data_;
  /*  Create a shared pointer to a simulator interface The interface
      need to have all the required data about the quadcopter*/
  std::shared_ptr<simulator_t> sim_interface_;
  std::vector<lt::position3D<double>>::iterator leader_;
  std::vector<lt::position3D<double>>::iterator follower_;
};

#include "state.hxx"
