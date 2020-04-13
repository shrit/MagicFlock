#pragma once

#include <vector>
#include <mlpack/core.hpp>

#include "global.hh"

namespace lt = local_types;

class Flocking
{

public:
  Flocking(double sepGain,
           double cohGain,
           double migGain,
           double cutoffDist,
           std::vector<lt::position3D<double>> position_of_neighbors);

  arma::vec cohesionVelocity();
  arma::vec separationVelocity();
  arma::vec migrationVelocity();
  arma::vec reynoldsVeclocity();
  arma::vec Velocity();

  Flocking(Flocking const&) = delete;
  Flocking(Flocking&&) = default;

private:
  double separation_gain_;
  double cohesion_gain_;
  double migration_gain_;
  double cutoff_distance_;
  int number_of_neighbors_;
};
