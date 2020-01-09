#include "include/action.hh"

Actions::Actions()
  : distribution_int_(0, 5)
  , distribution_real_(0, 1)
  , generator_(random_dev())
{}

std::vector<Actions::Action>
Actions::ActionConstructor(arma::mat values)
{
  std::vector<Actions::Action> actions;
  for (arma::uword i = 0; i < values.n_rows; ++i) {
    arma::rowvec action_row = values.row(i);
    std::vector<double> action = mtools_.to_std_vector(action_row);
    int action_value = mtools_.from_one_hot_encoding(action);
    actions.push_back(static_cast<Actions::Action>(action_value));
  }
  return actions;
}

std::string
Actions::action_to_str(Action action)
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

/* Get the best action from the model according to the best values */
Actions::Action
Actions::extract_action_from_index(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  Action action = Action::Unknown;
  /*  Access matrix values according to a given index  */
  /*  Only one action exist that equal 1 in each row of
   a matrix */
  if (features(index, 14) == 1) {
    action = Action::forward;
  } else if (features(index, 15) == 1) {
    action = Action::backward;
  } else if (features(index, 16) == 1) {
    action = Action::left;
  } else if (features(index, 17) == 1) {
    action = Action::right;
  } else if (features(index, 18) == 1) {
    action = Action::up;
  } else if (features(index, 19) == 1) {
    action = Action::down;
  } else if (features(index, 20) == 1) {
    action = Action::NoMove;
  }
  return action;
}

Actions::Action
Actions::int_to_action(int action_value)
{
  Action action;
  return action = static_cast<Action>(action_value);
}

std::vector<Actions::Action>
Actions::all_possible_actions() const
{
  return possible_actions_;
}

Actions::Action
Actions::random_action_generator()
{
  int random_action = distribution_int_(generator_);
  Action action = static_cast<Action>(random_action);
  return action;
}

double
Actions::generate_real_random()
{
  double random_double = distribution_real_(generator_);
  return random_double;
}
/*  The condition is to generate an action that is not the same to the
    parameter action and not opposed to this action. This is more
    comfortable since the opposed action apply high noise on traveled
    distance. Also this is more logic, since allow more variability in
    the data set */
Actions::Action
Actions::random_action_generator_with_all_conditions(Action action)
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
    high noise on traveled distance. Also this is more logic, since
    allow more variability in the data set */
Actions::Action
Actions::random_action_generator_with_only_opposed_condition(Action action)
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

Actions::Action
Actions::undo_action(Actions::Action action)
{
  Actions::Action undo_action = Actions::Action::Unknown;
  switch (action) {
    case Actions::Action::forward:
      undo_action = Actions::Action::backward;
      break;
    case Actions::Action::backward:
      undo_action = Actions::Action::forward;
      break;
    case Actions::Action::left:
      undo_action = Actions::Action::right;
      break;
    case Actions::Action::right:
      undo_action = Actions::Action::left;
      break;
    case Actions::Action::up:
      undo_action = Actions::Action::down;
      break;
    case Actions::Action::down:
      undo_action = Actions::Action::up;
      break;
    case Actions::Action::NoMove:
      undo_action = Actions::Action::NoMove;
      break;
    case Actions::Action::Unknown:
      undo_action = Actions::Action::Unknown;
      break;
  }
  return undo_action;
}

Actions::Action
Actions::pair_action_bob(Actions::Action action)
{
  Actions::Action pair_action = Actions::Action::Unknown;
  switch (action) {
    case Actions::Action::forward:
      pair_action = Actions::Action::right;
      break;
    case Actions::Action::backward:
      pair_action = Actions::Action::left;
      break;
    case Actions::Action::left:
      pair_action = Actions::Action::backward;
      break;
    case Actions::Action::right:
      pair_action = Actions::Action::forward;
      break;
    case Actions::Action::up:
      pair_action = Actions::Action::up;
      break;
    case Actions::Action::down:
      pair_action = Actions::Action::down;
      break;
    case Actions::Action::NoMove:
      pair_action = Actions::Action::NoMove;
      break;
    case Actions::Action::Unknown:
      pair_action = Actions::Action::Unknown;
      break;
  }
  return pair_action;
}

Actions::Action
Actions::pair_action_charlie(Actions::Action action)
{
  Actions::Action pair_action = Actions::Action::Unknown;
  switch (action) {
    case Actions::Action::forward:
      pair_action = Actions::Action::left;
      break;
    case Actions::Action::backward:
      pair_action = Actions::Action::right;
      break;
    case Actions::Action::left:
      pair_action = Actions::Action::forward;
      break;
    case Actions::Action::right:
      pair_action = Actions::Action::backward;
      break;
    case Actions::Action::up:
      pair_action = Actions::Action::up;
      break;
    case Actions::Action::down:
      pair_action = Actions::Action::down;
      break;
    case Actions::Action::NoMove:
      pair_action = Actions::Action::NoMove;
      break;
    case Actions::Action::Unknown:
      pair_action = Actions::Action::Unknown;
      break;
  }
  return pair_action;
}

std::tuple<Actions::Action, Actions::Action>
Actions::deduce_action_from_distance(double distance_t_1_b,
                                     double distance_t_b,
                                     Actions::Action b_last_action,
                                     Actions::Action b_before_2_last_action,
                                     double distance_t_1_c,
                                     double distance_t_c,
                                     Actions::Action c_last_action,
                                     Actions::Action c_before_2_last_action,
                                     double alti_diff_t,
                                     double& score_b,
                                     double& score_c)
{
  Actions::Action action_b = Action::Unknown;
  Actions::Action action_c = Action::Unknown;

  if (score_b == 0) {
    action_b = undo_action(b_last_action);
    score_b = 1;

  } else if (score_c == 0) {
    action_c = undo_action(c_last_action);
    score_c = 1;

  } else if (score_b == 1) {
    action_b = pair_action_bob(b_before_2_last_action);
    score_b = -1;
  } else if (score_c == 1) {
    action_c = pair_action_charlie(c_before_2_last_action);
    score_c = -1;
  } else {

    if (distance_t_b > distance_t_1_b and distance_t_c > distance_t_1_c and
        std::fabs(alti_diff_t) < 0.4) {
      if (generate_real_random() > 0.5) {
        action_b = Action::forward;
        action_c = Action::left;
      } else {
        action_b = Action::right;
        action_c = Action::forward;
      }

    } else if (distance_t_b > distance_t_1_b and
               distance_t_c < distance_t_1_c and std::fabs(alti_diff_t) < 0.4) {
      if (generate_real_random() > 0.5) {
        action_b = Action::right;
        action_c = Action::backward;
      } else {
        action_b = Action::forward;
        action_c = Action::right;
      }
    } else if (distance_t_b < distance_t_1_b and
               distance_t_c < distance_t_1_c and std::fabs(alti_diff_t) < 0.4) {
      if (generate_real_random() > 0.5) {
        action_b = Action::backward;
        action_c = Action::right;
      } else {
        action_b = Action::left;
        action_c = Action::backward;
      }
    } else if (distance_t_b < distance_t_1_b and
               distance_t_c > distance_t_1_c and std::fabs(alti_diff_t) < 0.4) {
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
  }
  return std::make_tuple(action_b, action_c);
}

Actions::Action
Actions::deduce_oracle_action_from_distance(double distance_t_1_b,
                                            double distance_t_b,
                                            double distance_t_1_c,
                                            double distance_t_c,
                                            double alti_diff_t)
{
  Actions::Action action = Action::Unknown;
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

Actions::Action
Actions::generate_leader_action(bool change_leader_action,
                                bool stop_going_down,
                                double distance_to_b,
                                double distance_to_c,
                                Actions::Action last_action)
{
  Actions::Action leader_action;
  if (distance_to_b > 3.4 and distance_to_c > 3.4) {
    leader_action = Action::NoMove;
  } else if (change_leader_action == true) {
    leader_action =
      random_action_generator_with_only_opposed_condition(last_action);
  } else if (stop_going_down == true) {
    leader_action =
      random_action_generator_with_only_opposed_condition(last_action);
  } else {
    leader_action = last_action;
  }
  return leader_action;
}
