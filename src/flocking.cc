#include "include/flocking.hh"

Flocking::Flocking(double sepGain,
                   double cohGain,
                   double migGain,
                   double cutoffDist,
                   std::vector<lt::position3D<double>> position_of_neighbors)
  : separation_gain_(sepGain)
  , cohesion_gain_(cohGain)
  , migration_gain_(migGain)
  , cutoff_distance_(cutoffDist)

{
  number_of_neighbors_ = position_of_neighbors.size();
  
}

arma::vec
Flocking::cohesionVelocity()
{
 double param =  cohesion_gain_/ number_of_neighbors_;
  
}

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
