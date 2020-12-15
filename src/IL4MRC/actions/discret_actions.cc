#include "discret_actions.hpp"

DiscretActions::DiscretActions()
  : distribution_int_(0, 5)
  , distribution_real_(0, 1)
  , generator_(random_dev())
{}

DiscretActions::Action
DiscretActions::action() const
{
  return action_;
}

DiscretActions::Action&
DiscretActions::action()
{
  return action_;
}

arma::colvec
DiscretActions::Data()
{
  return one_hot_.to_one_hot_encoding(action_, 7);
}

std::string
DiscretActions::DiscretActions::action_to_str(Action action)
{
  std::string string_action = "";
  switch (action) {
    case Action::forward:
      string_action = "Forward";
      break;
    case Action::backward:
      string_action = "Backward";
      break;
    case Action::left:
      string_action = "Left";
      break;
    case Action::right:
      string_action = "Right";
      break;
    case Action::up:
      string_action = "Up";
      break;
    case Action::down:
      string_action = "Down";
      break;
    case Action::NoMove:
      string_action = "NoMove";
      break;
    case Action::Unknown:
      string_action = "Unknown";
      break;
  }
  return string_action;
}

DiscretActions::Action
DiscretActions::int_to_action(int action_value)
{
  Action action;
  return action = static_cast<Action>(action_value);
}

std::vector<DiscretActions::Action>
DiscretActions::all_possible_actions() const
{
  return possible_actions_;
}

DiscretActions::Action
DiscretActions::random_action_generator()
{
  int random_action = distribution_int_(generator_);
  Action action = static_cast<Action>(random_action);
  return action;
}

double
DiscretActions::generate_real_random()
{
  double random_double = distribution_real_(generator_);
  return random_double;
}


