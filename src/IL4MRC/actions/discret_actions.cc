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
/*  The condition is to generate an action that is not the same to the
    parameter action and not opposed to this action. This is more
    comfortable since the opposed action apply high noise on traveled
    distance. Also this is more logical, since we allow more variability in
    the data set */
DiscretActions::Action
DiscretActions::random_action_generator_with_all_conditions(Action action)
{
  Action action_ = Action::Unknown;

  if (action == Action::backward) {
    action_ = random_action_generator();
    while (action_ == Action::forward or action_ == Action::backward) {
      action_ = random_action_generator();
    }
  } else if (action == Action::down) {
    action_ = random_action_generator();
    while (action_ == Action::up or action_ == Action::down) {
      action_ = random_action_generator();
    }
  } else if (action == Action::up) {
    action_ = random_action_generator();
    while (action_ == Action::down or action_ == Action::up) {
      action_ = random_action_generator();
    }
  } else if (action == Action::forward) {
    action_ = random_action_generator();
    while (action_ == Action::backward or action_ == Action::forward) {
      action_ = random_action_generator();
    }
  } else if (action == Action::right) {
    action_ = random_action_generator();
    while (action_ == Action::left or action_ == Action::right) {
      action_ = random_action_generator();
    }
  } else if (action == Action::left) {
    action_ = random_action_generator();
    while (action_ == Action::right or action_ == Action::left) {
      action_ = random_action_generator();
    }
  } else if (action == Action::Unknown or action == Action::NoMove) {
    action_ = random_action_generator();
  }
  return action_;
}

/*  The condition is to generate an action that not opposed to this
    action. This is more comfortable since the opposed action apply
    high noise on traveled distance. Also this is more logical, since
    we allow more variability in the data set */
DiscretActions::Action
DiscretActions::random_action_generator_with_only_opposed_condition(
  Action action)
{
  Action action_ = Action::Unknown;

  if (action == Action::backward) {
    action_ = random_action_generator();
    while (action_ == Action::forward) {
      action_ = random_action_generator();
    }
  } else if (action == Action::down) {
    action_ = random_action_generator();
    while (action_ == Action::up) {
      action_ = random_action_generator();
    }
  } else if (action == Action::up) {
    action_ = random_action_generator();
    while (action_ == Action::down) {
      action_ = random_action_generator();
    }
  } else if (action == Action::forward) {
    action_ = random_action_generator();
    while (action_ == Action::backward) {
      action_ = random_action_generator();
    }
  } else if (action == Action::right) {
    action_ = random_action_generator();
    while (action_ == Action::left) {
      action_ = random_action_generator();
    }
  } else if (action == Action::left) {
    action_ = random_action_generator();
    while (action_ == Action::right) {
      action_ = random_action_generator();
    }
  } else if (action == Action::Unknown or action == Action::NoMove) {
    action_ = random_action_generator();
  }
  return action_;
}

DiscretActions::Action
DiscretActions::undo_action(DiscretActions::Action action)
{
  DiscretActions::Action undo_action = DiscretActions::Action::Unknown;
  switch (action) {
    case DiscretActions::Action::forward:
      undo_action = DiscretActions::Action::backward;
      break;
    case DiscretActions::Action::backward:
      undo_action = DiscretActions::Action::forward;
      break;
    case DiscretActions::Action::left:
      undo_action = DiscretActions::Action::right;
      break;
    case DiscretActions::Action::right:
      undo_action = DiscretActions::Action::left;
      break;
    case DiscretActions::Action::up:
      undo_action = DiscretActions::Action::down;
      break;
    case DiscretActions::Action::down:
      undo_action = DiscretActions::Action::up;
      break;
    case DiscretActions::Action::NoMove:
      undo_action = DiscretActions::Action::NoMove;
      break;
    case DiscretActions::Action::Unknown:
      undo_action = DiscretActions::Action::Unknown;
      break;
  }
  return undo_action;
}

std::tuple<DiscretActions::Action, DiscretActions::Action>
DiscretActions::generate_followers_action_using_distance(double distance_t_1_b,
                                                         double distance_t_b,
                                                         double distance_t_1_c,
                                                         double distance_t_c,
                                                         double alti_diff_t)
{
  DiscretActions::Action action_b = Action::NoMove;
  DiscretActions::Action action_c = Action::NoMove;

  if (distance_t_b > distance_t_1_b and distance_t_c > distance_t_1_c and
      std::fabs(alti_diff_t) < 0.4) {
    if (generate_real_random() > 0.5) {
      action_b = Action::forward;
      action_c = Action::left;
    } else {
      action_b = Action::right;
      action_c = Action::forward;
    }

  } else if (distance_t_b > distance_t_1_b and distance_t_c < distance_t_1_c and
             std::fabs(alti_diff_t) < 0.4) {
    if (generate_real_random() > 0.5) {
      action_b = Action::right;
      action_c = Action::backward;
    } else {
      action_b = Action::forward;
      action_c = Action::right;
    }
  } else if (distance_t_b < distance_t_1_b and distance_t_c < distance_t_1_c and
             std::fabs(alti_diff_t) < 0.4) {
    if (generate_real_random() > 0.5) {
      action_b = Action::backward;
      action_c = Action::right;
    } else {
      action_b = Action::left;
      action_c = Action::backward;
    }
  } else if (distance_t_b < distance_t_1_b and distance_t_c > distance_t_1_c and
             std::fabs(alti_diff_t) < 0.4) {
    if (generate_real_random() > 0.5) {
      action_b = Action::left;
      action_c = Action::forward;
    } else {
      action_b = Action::backward;
      action_c = Action::left;
    }
  } else if ((alti_diff_t) > 0.7) {
    action_b = Action::up;
    action_c = Action::up;
  } else {
    action_b = Action::down;
    action_c = Action::down;
  }
  return std::make_tuple(action_b, action_c);
}

DiscretActions::Action
DiscretActions::generate_follower_action_using_oracle(double distance_t_1_b,
                                                      double distance_t_b,
                                                      double distance_t_1_c,
                                                      double distance_t_c,
                                                      double alti_diff_t)
{
  DiscretActions::Action action = Action::Unknown;
  if (distance_t_b > distance_t_1_b and distance_t_c > distance_t_1_c and
      std::fabs(alti_diff_t) < 0.4) {
    action = Action::forward;

  } else if (distance_t_b > distance_t_1_b and distance_t_c < distance_t_1_c and
             std::fabs(alti_diff_t) < 0.4) {
    action = Action::right;

  } else if (distance_t_b < distance_t_1_b and distance_t_c < distance_t_1_c and
             std::fabs(alti_diff_t) < 0.4) {
    action = Action::backward;

  } else if (distance_t_b < distance_t_1_b and distance_t_c > distance_t_1_c and
             std::fabs(alti_diff_t) < 0.4) {
    action = Action::left;

  } else if ((alti_diff_t) > 0.7) {
    action = Action::up;
  } else {
    action = Action::down;
  }
  return action;
}

DiscretActions::Action
DiscretActions::generate_leader_action(bool change_leader_action,
                                       bool stop_going_down,
                                       DiscretActions::Action last_action,
                                       DiscretActions::Action current_action)
{
  DiscretActions::Action leader_action = DiscretActions::Action::Unknown;

  if (change_leader_action == true) {
    leader_action =
      random_action_generator_with_only_opposed_condition(last_action);
  } else if (stop_going_down == true) {
    while (leader_action == DiscretActions::Action::down) {
      leader_action =
        random_action_generator_with_only_opposed_condition(last_action);
    }
  } else if (last_action == DiscretActions::Action::Unknown) {
    leader_action = current_action;

  } else {
    leader_action = last_action;
  }
  return leader_action;
}

DiscretActions::Action
DiscretActions::validate_leader_action(double distance_to_b,
                                       double distance_to_b_1,
                                       double distance_to_c,
                                       double distance_to_c_1,
                                       DiscretActions::Action current_action)
{
  DiscretActions::Action leader_action = current_action;
  if (std::fabs(distance_to_b - distance_to_b_1) > 0.80 or
      std::fabs(distance_to_c - distance_to_c_1) > 0.80) {
    leader_action = Action::NoMove;
  }
  return leader_action;
}

DiscretActions::Action
DiscretActions::validate_followers_action(
  std::vector<double> current_distances,
  std::vector<double> last_distances,
  DiscretActions::Action before_2_last_action,
  DiscretActions::Action current_action)
{
  DiscretActions::Action follower_action = current_action;
  if (before_2_last_action == DiscretActions::Action::Unknown) {
    follower_action = current_action;
  } else if (std::fabs(current_distances.at(0) - last_distances.at(0)) > 0.80 or
             std::fabs(current_distances.at(1) - last_distances.at(1)) > 0.80) {
    follower_action = undo_action(before_2_last_action);
  }
  return follower_action;
}
