#include "continuous_actions.hpp"

ContinuousActions::ContinuousActions()
  : velocity_vector_(0, 0, 0)
  , data_(3, arma::fill::zeros)
{
  arma::mat all_possible_actions(3, 1000, arma::fill::zeros);
  all_possible_actions_ = all_possible_actions;
  calculate_all_possible_actions();
}

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

ignition::math::Vector3d
ContinuousActions::leader_action() const
{
  return leader_velocity_vector_;
}

ignition::math::Vector3d&
ContinuousActions::leader_action()
{
  return leader_velocity_vector_;
}

ignition::math::Vector3d
ContinuousActions::followers_action() const
{
  return followers_velocity_vector_;
}

ignition::math::Vector3d&
ContinuousActions::followers_action()
{
  return followers_velocity_vector_;
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
ContinuousActions::to_action(arma::uword index)
{
  ContinuousActions action;
  ignition::math::Vector3d velocity;
  action.set_action(all_possible_actions().col(index));
  return action;
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

void
ContinuousActions::calculate_all_possible_actions()
{
  arma::uword i = 0;
  for (arma::uword x = 0; x < 10; x++) {
    for (arma::uword y = 0; y < 10; y++) {
      for (arma::uword z = 0; z < 10; z++) {
        all_possible_actions_(0, i) = (x * 0.1);
        all_possible_actions_(1, i) = (y * 0.1);
        all_possible_actions_(2, i) = (z * 0.1);
        ++i;
      }
    }
  }
}

arma::mat
ContinuousActions::all_possible_actions() const
{
  return all_possible_actions_;
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

arma::colvec
ContinuousActions::leader_data()
{
  leader_data_.at(0) = leader_velocity_vector_.X();
  leader_data_.at(1) = leader_velocity_vector_.Y();
  leader_data_.at(2) = leader_velocity_vector_.Z();
  return leader_data_;
}

arma::colvec
ContinuousActions::followers_data()
{
  followers_data_.at(0) = followers_velocity_vector_.X();
  followers_data_.at(1) = followers_velocity_vector_.Y();
  followers_data_.at(2) = followers_velocity_vector_.Z();
  return followers_data_;
}
