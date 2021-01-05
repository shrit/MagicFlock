#include "discret_actions.hpp"

DiscretActions::DiscretActions()
  : velocity_vector_(0, 0, 0)
  , leader_velocity_vector_(0, 0, 0)
  , followers_velocity_vector_(0, 0, 0)
  , data_(3, arma::fill::zeros)
  , leader_data_(3, arma::fill::zeros)
  , followers_data_(3, arma::fill::zeros)
{}

ignition::math::Vector3d
DiscretActions::action() const
{
  return velocity_vector_;
}

ignition::math::Vector3d&
DiscretActions::action()
{
  return velocity_vector_;
}

ignition::math::Vector3d
DiscretActions::leader_action() const
{
  return leader_velocity_vector_;
}

ignition::math::Vector3d&
DiscretActions::leader_action()
{
  return leader_velocity_vector_;
}

ignition::math::Vector3d
DiscretActions::followers_action() const
{
  return followers_velocity_vector_;
}

ignition::math::Vector3d&
DiscretActions::followers_action()
{
  return followers_velocity_vector_;
}

void
DiscretActions::set_action(arma::colvec data)
{
  velocity_vector_.X() = data.at(0);
  velocity_vector_.Y() = data.at(1);
  velocity_vector_.Z() = data.at(2);
}

arma::colvec
DiscretActions::Data()
{
  return one_hot_.to_one_hot_encoding(velocity_vector_, 9);
}

arma::colvec
DiscretActions::leader_data()
{
  return one_hot_.to_one_hot_encoding(leader_velocity_vector_, 9);
}

arma::colvec
DiscretActions::followers_data()
{
  return one_hot_.to_one_hot_encoding(followers_velocity_vector_, 9);
}

arma::mat
DiscretActions::all_possible_actions()
{
  arma::mat all_possible_actions{
    { 1, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 1, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 1, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 1, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 1, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 1, 0, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 1, 0 },
    { 0, 0, 0, 0, 0, 0, 0, 0, 1 },
  };

  return all_possible_actions;
}

int
DiscretActions::action_to_int(ignition::math::Vector3d action)
{
  int index = one_hot_.to_one_hot_encoding(action, 9).index_max();
  return index;
}

DiscretActions
DiscretActions::int_to_action(arma::uword index)
{
  DiscretActions action;
  ignition::math::Vector3d velocity{ 0, 0, 0 };

  /* This is not a nice solution but resolve the issue for now */
  if (index == all_possible_actions().col(0).index_max()) {
    velocity.X() = 1;
  } else if (index == all_possible_actions().col(1).index_max()) {
    velocity.X() = -1;
  } else if (index == all_possible_actions().col(2).index_max()) {
    velocity.Y() = 1;
  } else if (index == all_possible_actions().col(3).index_max()) {
    velocity.Y() = -1;
  // Comment up and down for now
  // } else if (index == all_possible_actions().col(4).index_max()) {
  //   velocity.Z() = 1;
  // } else if (index == all_possible_actions().col(5).index_max()) {
  //   velocity.Z() = -1;
  } else if (index == all_possible_actions().col(4).index_max()) {
    velocity.X() = 1;
    velocity.Y() = 1;
  } else if (index == all_possible_actions().col(5).index_max()) {
    velocity.X() = -1;
    velocity.Y() = -1;
  } else if (index == all_possible_actions().col(6).index_max()) {
    velocity.X() = 1;
    velocity.Y() = -1;
  } else if (index == all_possible_actions().col(7).index_max()) {
    velocity.X() = -1;
    velocity.Y() = 1;
  } else if (index == all_possible_actions().col(8).index_max()) {
  }
  action.action() = velocity;
  return action;
}

int
DiscretActions::random_action_generator()
{
  // int random_action = distribution_int_(generator_);
  // Action action = static_cast<Action>(random_action);
  // return action;
}
