#include "continuous_actions.hpp"

ContinuousActions::ContinuousActions()
  : velocity_vector_(0, 0, 0)
  , leader_velocity_vector_(0, 0, 0)
  , followers_velocity_vector_(0, 0, 0) 
  , data_(3, arma::fill::zeros)
  , leader_data_(3, arma::fill::zeros)
  , followers_data_(3, arma::fill::zeros)
{
 // calculate_all_possible_actions();
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

/*
 * I am not sure of the evolution of the functions, I am commenting
 * them until I have better insights on what to do
 */

// ContinuousActions
// ContinuousActions::to_action(arma::uword index)
// {
//   ContinuousActions action;
//   ignition::math::Vector3d velocity;
//   action.set_action(all_possible_actions().col(index));
//   return action;
// }

// void
// ContinuousActions::calculate_all_possible_actions()
// {
//   arma::uword i = 0;
//   for (arma::uword x = 0; x < 10; x++) {
//     for (arma::uword y = 0; y < 10; y++) {
//       for (arma::uword z = 0; z < 10; z++) {
//         all_possible_actions_(0, i) = (x * 0.1);
//         all_possible_actions_(1, i) = (y * 0.1);
//         all_possible_actions_(2, i) = (z * 0.1);
//         ++i;
//       }
//     }
//   }
// }

void
ContinuousActions::set_action(arma::colvec data)
{
  velocity_vector_.X() = data.at(0);
  velocity_vector_.Y() = data.at(1);
  velocity_vector_.Z() = data.at(2);
}

void
ContinuousActions::set_action_leader(arma::colvec data)
{
  leader_velocity_vector_.X() = data.at(0);
  leader_velocity_vector_.Y() = data.at(1);
  leader_velocity_vector_.Z() = data.at(2);
}

void
ContinuousActions::set_action_followers(arma::colvec data)
{
  followers_velocity_vector_.X() = data.at(0);
  followers_velocity_vector_.Y() = data.at(1);
  followers_velocity_vector_.Z() = data.at(2);
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
