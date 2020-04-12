#pragma once


class Flocking
{

public:
  
  Flocking(double sepGain, double cohGain, double migGain);

  arma::vec cohesionVelocity();
  arma::vec separationVelocity();
  arma::vec migrationVelocity();
  arma::vec reynoldsVeclocity();
  arma::vec Velocity();

  Flocking(Flocking const&) = delete;
  Flocking(Flocking&&) = default;

private:
  double separation_gain;
  double cohesion_gain;
  double migration_gain;
  
};
