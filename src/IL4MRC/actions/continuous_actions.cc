#include "continuous_actions.hpp"

ContinuousActions::ContinuousActions()
  : velocity_vector_(0 ,0, 0)
  , data_(3, arma::fill::zeros)
{}

ignition::math::Vector3d
ContinuousActions::action() const
{
  return velocity_vector_;
}

ignition::math::Vector3d&
ContinuousActions::action()
{
  return velocity_vector_;
}

void
ContinuousActions::set_action(arma::colvec data)
{ 
  velocity_vector_.X() = data.at(0);
  velocity_vector_.Y() = data.at(1);
  velocity_vector_.Z() = data.at(2);
}

arma::colvec
ContinuousActions::Data()
{ 
  data_.at(0) = velocity_vector_.X();
  data_.at(1) = velocity_vector_.Y();
  data_.at(2) = velocity_vector_.Z();  
  return data_;
}
