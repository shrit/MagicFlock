#include "continuous_actions.hpp"

ContinuousActions::ContinuousActions()
  : velocity_vector_(0, 0, 0)
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
std::vector<int>
ContinuousActions::one_hot_action() const
{
  return one_hot_encoding_action_;
}

std::vector<int>&
ContinuousActions::one_hot_action()
{
  return one_hot_encoding_action_;
}

arma::mat
ContinuousActions::all_possible_actions()
{
  arma::mat all_possible_actions 
  {{1,0,0,0,0,0,0,0,0,0,0,0,0},
   {0,1,0,0,0,0,0,0,0,0,0,0,0},
   {0,0,1,0,0,0,0,0,0,0,0,0,0},
   {0,0,0,1,0,0,0,0,0,0,0,0,0},
   {0,0,0,0,1,0,0,0,0,0,0,0,0},
   {0,0,0,0,0,1,0,0,0,0,0,0,0},
   {0,0,0,0,0,0,1,0,0,0,0,0,0},
   {0,0,0,0,0,0,0,1,0,0,0,0,0},
   {0,0,0,0,0,0,0,0,1,0,0,0,0},
   {0,0,0,0,0,0,0,0,0,1,0,0,0},
   {0,0,0,0,0,0,0,0,0,0,1,0,0},
   {0,0,0,0,0,0,0,0,0,0,0,1,0},
   {0,0,0,0,0,0,0,0,0,0,0,0,1},};
 
  return all_possible_actions;
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
