#pragma once

class Flocking
{

public:
  Flocking(double sepGain, double cohGain, double migGain, double cutoffDist);

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
};
