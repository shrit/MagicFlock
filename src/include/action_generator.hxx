#pragma once

template<class QuadrotorType>
ActionGenerator<QuadrotorType>::ActionGenerator(
  typename std::vector<QuadrotorType>::iterator quad)
  : quad_(quad)
{
  // Nothing to do here
}

template<class QuadrotorType>
DiscretActions::Action
ActionGenerator<QuadrotorType>::generate_random_action()
{
  int random_action = distribution_int_(generator_);
  Action action = static_cast<Action>(random_action);
  return action;
}

template<class QuadrotorType>
DiscretActions::Action
ActionGenerator<QuadrotorType>::generate_persistant_action(int for_n_timestep,
                                                           int timesteps)
{
  DiscretActions::Action action = DiscretActions::Action::Unknown;

  if (timesteps % for_n_timestep == 0) {
    action = random_action_generator_with_only_opposed_condition(
      quad_->current_action());
  } else {
    action = quad_->current_action();
  }

  if (quad_->height() < 15) {
    if (quad_->current_action() == DiscretActions::Action::down)
      action = random_action_generator_with_only_opposed_condition(
        quad_->current_action());
  }
  return action;
}

/*  The function  generates an action that not opposed to the proposed one.
 * This is more comfortable since the opposed action apply
 * high noise on traveled distance. Also this is more logic, since
 * allow more variability in the data set
 * */
template<class QuadrotorType>
DiscretActions::Action
ActionGenerator<QuadrotorType>::generate_random_action_no_opposed(
  DiscretActions::Action action)
{
  DiscretActions::Action action_ = DiscretActions::Action::Unknown;

  if (action == DiscretActions::Action::backward) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::forward) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::down) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::up) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::up) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::down) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::forward) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::backward) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::right) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::left) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::left) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::right) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::Unknown or
             action == DiscretActions::Action::NoMove) {
    action_ = generate_random_action();
  }
  return action_;
}

/*  The condition is to generate an action that is not the same to the
     parameter action and not opposed to this action. This is more
     comfortable since the opposed action apply high noise on traveled
     distance. Also this is more logic, since allow more variability in
     the data set */
template<class QuadrotorType>
DiscretActions::Action
ActionGenerator<QuadrotorType>::generate_random_action_no_opposed_no_same(
  DiscretActions::Action action)
{
  DiscretActions::Action action_ = DiscretActions::Action::Unknown;

  if (action == DiscretActions::Action::backward) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::forward or
           action_ == DiscretActions::Action::backward) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::down) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::up or action_ == DiscretActions::Action::down) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::up) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::down or action_ == DiscretActions::Action::up) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::forward) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::backward or
           action_ == DiscretActions::Action::forward) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::right) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::left or
           action_ == DiscretActions::Action::right) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::left) {
    action_ = generate_random_action();
    while (action_ == DiscretActions::Action::right or
           action_ == DiscretActions::Action::left) {
      action_ = generate_random_action();
    }
  } else if (action == DiscretActions::Action::Unknown or
             action == DiscretActions::Action::NoMove) {
    action_ = generate_random_action();
  }
  return action_;
}

template<class QuadrotorType>
DiscretActions::Action
ActionGenerator<QuadrotorType>::generate_action_from_oracle()
{
  DiscretActions::Action action = DiscretActions::Action::Unknown;

  if (quad_->id() == 1) {
    if (quad_->current_state().distance_to(0) >
          quad_->last_state().distance_to(0) and
        quad_->current_state().distance_to(3) >
          quad_->last_state().distance_to(3) and
        quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::left;

    } else if (quad_->current_state().distance_to(0) >
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::forward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::backward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::right;
    } else if (quad_->height_difference() > 0.7) {
      action = DiscretActions::Action::up;
    } else {
      action = DiscretActions::Action::down;
    }
  } else if (quad_->id() == 2) {
    if (quad_->current_state().distance_to(0) >
          quad_->last_state().distance_to(0) and
        quad_->current_state().distance_to(3) >
          quad_->last_state().distance_to(3) and
        quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::right;

    } else if (quad_->current_state().distance_to(0) >
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::forward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::backward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = DiscretActions::Action::left;
    } else if (quad_->height_difference() > 0.7) {
      action = DiscretActions::Action::up;
    } else {
      action = DiscretActions::Action::down;
    }
  }
  return action;
}
