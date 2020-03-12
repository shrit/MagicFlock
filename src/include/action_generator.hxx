#pragma once

template<class QuadrotorType>
ActionGenerator<QuadrotorType>::ActionGenerator(
  typename std::vector<QuadrotorType>::iterator quad)
  : quad_(quad)
{
  // Nothing to do here
}

template<class QuadrotorType>
Actions::Action
ActionGenerator<QuadrotorType>::generate_random_action()
{
  int random_action = distribution_int_(generator_);
  Action action = static_cast<Action>(random_action);
  return action;
}

template<class QuadrotorType>
Actions::Action
ActionGenerator<QuadrotorType>::generate_persistant_action(int for_n_timestep,
                                                         int timesteps)
{
  Actions::Action action = Actions::Action::Unknown;

  if (timesteps % for_n_timestep == 0) {
    action = random_action_generator_with_only_opposed_condition(
      quad_->current_action());
  } else {
    action = quad_->current_action();
  }

  if (quad_->height() < 15) {
    if (quad_->current_action() == Actions::Action::down)
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
Actions::Action
ActionGenerator<QuadrotorType>::generate_random_action_no_opposed(
  Actions::Action action)
{
  Actions::Action action_ = Actions::Action::Unknown;

  if (action == Actions::Action::backward) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::forward) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::down) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::up) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::up) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::down) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::forward) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::backward) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::right) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::left) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::left) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::right) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::Unknown or
             action == Actions::Action::NoMove) {
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
Actions::Action
ActionGenerator<QuadrotorType>::generate_random_action_no_opposed_no_same(
  Actions::Action action)
{
  Actions::Action action_ = Actions::Action::Unknown;

  if (action == Actions::Action::backward) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::forward or
           action_ == Actions::Action::backward) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::down) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::up or action_ == Actions::Action::down) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::up) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::down or action_ == Actions::Action::up) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::forward) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::backward or
           action_ == Actions::Action::forward) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::right) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::left or
           action_ == Actions::Action::right) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::left) {
    action_ = generate_random_action();
    while (action_ == Actions::Action::right or
           action_ == Actions::Action::left) {
      action_ = generate_random_action();
    }
  } else if (action == Actions::Action::Unknown or
             action == Actions::Action::NoMove) {
    action_ = generate_random_action();
  }
  return action_;
}

template<class QuadrotorType>
Actions::Action
ActionGenerator<QuadrotorType>::generate_action_from_oracle()
{
  Actions::Action action = Actions::Action::Unknown;

  if (quad_->id() == 1) {
    if (quad_->current_state().distance_to(0) >
          quad_->last_state().distance_to(0) and
        quad_->current_state().distance_to(3) >
          quad_->last_state().distance_to(3) and
        quad_->height_difference() < 0.4) {
      action = Actions::Action::left;

    } else if (quad_->current_state().distance_to(0) >
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Actions::Action::forward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Actions::Action::backward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Actions::Action::right;
    } else if (quad_->height_difference() > 0.7) {
      action = Actions::Action::up;
    } else {
      action = Actions::Action::down;
    }
  } else if (quad_->id() == 2) {
    if (quad_->current_state().distance_to(0) >
          quad_->last_state().distance_to(0) and
        quad_->current_state().distance_to(3) >
          quad_->last_state().distance_to(3) and
        quad_->height_difference() < 0.4) {
      action = Actions::Action::right;

    } else if (quad_->current_state().distance_to(0) >
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) <
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Actions::Action::forward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Actions::Action::backward;
    } else if (quad_->current_state().distance_to(0) <
                 quad_->last_state().distance_to(0) and
               quad_->current_state().distance_to(3) >
                 quad_->last_state().distance_to(3) and
               quad_->height_difference() < 0.4) {
      action = Actions::Action::left;
    } else if (quad_->height_difference() > 0.7) {
      action = Actions::Action::up;
    } else {
      action = Actions::Action::down;
    }
  }
  return action;
}
