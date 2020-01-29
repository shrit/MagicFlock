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
ActionGenerator::generate_persistant_action(int for_n_timestep);

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
} /*  The condition is to generate an action that is not the same to the
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
ActionGenerator::generate_action_from_oracle();
