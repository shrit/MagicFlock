#pragma once

template<class simulator_t>
ActionGenerator::ActionGenerator(
  typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  : quad_(quad)
{
  // Nothing to do here
}

template<class simulator_t>
Action
ActionGenerator::generate_random_action()
{
  int random_action = distribution_int_(generator_);
  Action action = static_cast<Action>(random_action);
  return action;
}

template<class simulator_t>
Action
ActionGenerator::generate_persistant_action(int for_n_timestep, int timesteps)
{
  Actions::Action action = Actions::Action::Unknown;

  if (timesteps % for_n_timestep == 0) {
    action =
      random_action_generator_with_only_opposed_condition(quad_->last_action());
  } else if (quad_->height() < 15) {
    action =
      random_action_generator_with_only_opposed_condition(quad_->last_action());
  } else {
    action = quad_->last_action();
  }
  return action;
}

/*  The function  generates an action that not opposed to the proposed one.
 * This is more comfortable since the opposed action apply
 * high noise on traveled distance. Also this is more logic, since
 * allow more variability in the data set
 * */
template<class simulator_t>
Action
ActionGenerator::generate_random_action_no_opposed(Action action);
{
  Action action_ = Action::Unknown;

  if (action == Action::backward) {
    action_ = generate_random_action();
    while (action_ == Action::forward) {
      action_ = generate_random_action();
    }
  } else if (action == Action::down) {
    action_ = generate_random_action();
    while (action_ == Action::up) {
      action_ = generate_random_action();
    }
  } else if (action == Action::up) {
    action_ = generate_random_action();
    while (action_ == Action::down) {
      action_ = generate_random_action();
    }
  } else if (action == Action::forward) {
    action_ = generate_random_action();
    while (action_ == Action::backward) {
      action_ = generate_random_action();
    }
  } else if (action == Action::right) {
    action_ = generate_random_action();
    while (action_ == Action::left) {
      action_ = generate_random_action();
    }
  } else if (action == Action::left) {
    action_ = generate_random_action();
    while (action_ == Action::right) {
      action_ = generate_random_action();
    }
  } else if (action == Action::Unknown or action == Action::NoMove) {
    action_ = generate_random_action();
  }
  return action_;
}

/*  The condition is to generate an action that is not the same to the
     parameter action and not opposed to this action. This is more
     comfortable since the opposed action apply high noise on traveled
     distance. Also this is more logic, since allow more variability in
     the data set */
template<class simulator_t>
Action
ActionGenerator::generate_random_action_no_opposed_no_same(Action action)
{
  Action action_ = Action::Unknown;

  if (action == Action::backward) {
    action_ = generate_random_action();
    while (action_ == Action::forward or action_ == Action::backward) {
      action_ = generate_random_action();
    }
  } else if (action == Action::down) {
    action_ = generate_random_action();
    while (action_ == Action::up or action_ == Action::down) {
      action_ = generate_random_action();
    }
  } else if (action == Action::up) {
    action_ = generate_random_action();
    while (action_ == Action::down or action_ == Action::up) {
      action_ = generate_random_action();
    }
  } else if (action == Action::forward) {
    action_ = generate_random_action();
    while (action_ == Action::backward or action_ == Action::forward) {
      action_ = generate_random_action();
    }
  } else if (action == Action::right) {
    action_ = generate_random_action();
    while (action_ == Action::left or action_ == Action::right) {
      action_ = generate_random_action();
    }
  } else if (action == Action::left) {
    action_ = generate_random_action();
    while (action_ == Action::right or action_ == Action::left) {
      action_ = generate_random_action();
    }
  } else if (action == Action::Unknown or action == Action::NoMove) {
    action_ = generate_random_action();
  }
  return action_;
}

template<class simulator_t>
Action
ActionGenerator::generate_action_from_oracle()
{
  Actions::Action action = Action::Unknown;

  if (quad_->id() == 1) {
    if (quad_->current_state().distance_to(0) >
          quad_->last_state().distance_to(0) and
        quad_->current_state().distance_to(3) >
          quad_->last_state().distance_to(3) and
        quad_->height_difference() < 0.4) {
      action = Action::left;

    } else if (quad_->current_state().distance_to(0) >
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Action::forward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Action::backward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Action::right;
    } else if (quad_->height_difference() > 0.7) {
      action = Action::up;
    } else {
      action = Action::down;
    }
  } else if (quad_->id() == 2) {
    if (quad_->current_state().distance_to(0) >
          quad_->last_state().distance_to(0) and
        quad_->current_state().distance_to(3) >
          quad_->last_state().distance_to(3) and
        quad_->height_difference() < 0.4) {
      action = Action::right;

    } else if (quad_->current_state().distance_to(0) >
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Action::forward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Action::backward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Action::left;
    } else if (quad_->height_difference() > 0.7) {
      action = Action::up;
    } else {
      action = Action::down;
    }
  }
  return action;
}
