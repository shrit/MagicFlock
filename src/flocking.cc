#include "include/flocking.hh"

Flocking::Flocking(double sepGain, double cohGain, double migGain)
  : separation_gain_(sepGain)
  , cohesion_gain_(cohGain)
  , migration_gain_(migGain)

{}

arma::vec
Flocking::cohesionVelocity()
{}

arma::vec
Flocking::separationVelocity()
{}

arma::vec
Flocking::migrationVelocity()
{}

arma::vec
Flocking::reynoldsVeclocity()
{
  return cohesionVelocity() + migrationVelocity();
}

arma::vec
Flocking::Velocity()
{
  return cohesionVelocity() + separationVelocity() + migrationVelocity();
}
