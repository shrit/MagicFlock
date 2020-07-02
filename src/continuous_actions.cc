#include "include/continuous_actions.hh"

ContinuousActions::ContinuousActions()
  : data_(3)
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

arma::colvec
ContinuousActions::Data()
{ 
  data_.at(0) = velocity_vector_.X();
  data_.at(1) = velocity_vector_.Y();
  data_.at(2) = velocity_vector_.Z();  
  return data_;
}
