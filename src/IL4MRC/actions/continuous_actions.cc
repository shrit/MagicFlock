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

ContinuousActions
ContinuousActions::int_to_action(arma::uword index)
{
  ContinuousActions action;
  ignition::math::Vector3d velocity;

  /* This is not a nice solution but resolve the issue for now */
  if (index == all_possible_actions().col(0).index_max()) {
    velocity.X() = 0.5;
  } else if (index == all_possible_actions().col(1).index_max()) {
    velocity.X() = -0.5;
  } else if (index == all_possible_actions().col(2).index_max()) {
    velocity.Y() = 0.5;
  } else if (index == all_possible_actions().col(3).index_max()) {
    velocity.Y() = -0.5;
  } else if (index == all_possible_actions().col(4).index_max()) {
    velocity.Z() = 0.1;
  } else if (index == all_possible_actions().col(5).index_max()) {
    velocity.Z() = -0.1;
  } else if (index == all_possible_actions().col(6).index_max()) {
    velocity.X() = 0.5;
  } else if (index == all_possible_actions().col(7).index_max()) {
    velocity.X() = -0.5;
  } else if (index == all_possible_actions().col(8).index_max()) {
    velocity.Y() = 0.5;
  } else if (index == all_possible_actions().col(9).index_max()) {
    velocity.Y() = -0.5;
  } else if (index == all_possible_actions().col(10).index_max()) {
    velocity.Z() = 0.1;
  } else if (index == all_possible_actions().col(11).index_max()) {
    velocity.Z() = -0.1;
  }

  action.action() = velocity;
  return action;
}

arma::mat
ContinuousActions::all_possible_actions_one_hot()
{
  arma::mat all_possible_actions{
    { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
  };

  return all_possible_actions;
}

arma::mat
ContinuousActions::all_possible_actions()
{
  arma::mat all_possible_actions(3, 1000, arma::fill::zeros);
  arma::uword i = 0;
  for (double x = 0.0; x < 1; x = x + 0.1) {
    for (double y = 0.0; y < 1; y = y + 0.1) {
      for (double z = 0.0; z < 1; z = z + 0.1) {
        all_possible_actions(0, i) = x;
        all_possible_actions(1, i) = y;
        all_possible_actions(2, i) = z;
        ++i;
      }
    }
  }

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
