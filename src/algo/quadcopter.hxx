#pragma once

# include "quadcopter.hh"

Quadrotor::Quadrotor()
  :distribution_int_(0, 5),
   generator_(random_dev())
{}

void Quadrotor::init()
{
  data_set_.init_dataset_directory();
}

Quadrotor::Reward Quadrotor::
action_evaluator(const lt::triangle<double>& old_dist,
		 const lt::triangle<double>& new_dist)
{
  LogInfo() << "F1 differences: " << std::fabs(old_dist.f1 - new_dist.f1);
  LogInfo() << "F2 differences: " << std::fabs(old_dist.f2 - new_dist.f2);

  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);

   Reward reward = Reward::Unknown;

  if (0.5  > diff_f1 + diff_f2 ) {
    reward = Reward::very_good;
  } else if ( 1.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 0.5 ) {
    reward = Reward::good;
  } else if ( 1.5  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.0 ) {
    reward = Reward::bad;
  } else if ( 2.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.5 ) {
    reward = Reward::very_bad;
  }
  return reward;
}

Quadrotor::Action Quadrotor::
int_to_action(int action_value)
{
  Action action;
  return action = static_cast<Action>(action_value);  
}

/* Get the best action from the model according to the best values */
Quadrotor::Action Quadrotor::
action_follower(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  Quadrotor::Action action = Quadrotor::Action::NoMove;
  /*  Access matrix values according to a given index  */
  /*  Only one action exist that equal 1 in each row of 
   a matrix */  
  if (features(index, 14) == 1) {
    action = Quadrotor::Action::forward;
  } else if (features(index, 15) == 1) {
    action = Quadrotor::Action::backward;
  } else if (features(index, 16) == 1) {
    action = Quadrotor::Action::left;
  } else if (features(index, 17) == 1) {
    action = Quadrotor::Action::right;
  } else if (features(index, 18) == 1) {
    action = Quadrotor::Action::up;
  } else if (features(index, 19) == 1) {
    action = Quadrotor::Action::down;
  }
  return action;
}

void Quadrotor::
save_controller_count(double value)
{ data_set_.save_count_file(value); }

std::vector<Quadrotor::Action> Quadrotor::
possible_actions() const
{ return possible_actions_; }

Quadrotor::Action Quadrotor::
random_action_generator()
{
  int random_action = distribution_int_(generator_);
  Action action = static_cast<Action>(random_action);  
  return action;
}

/*  The condition is to generate an action that is not the same to the
    parameter action and not opposed to this action. This is more
    comfortable since the opposed action apply high noise on traveled
    distance. Also this is more logic, since allow more variability in
    the data set */
Quadrotor::Action Quadrotor::
random_action_generator_with_all_conditions(Action action)
{
  Action action_ = Action::NoMove;
  
  if (action == Action::backward) {
    action_ = random_action_generator();
    while (action_ == Action::forward or
	   action_ == Action::backward ) {
      action_ = random_action_generator();
    }
  } else if (action == Action::down) {
    action_ = random_action_generator();
    while (action_ == Action::up or
	   action_ == Action::down) {
      action_ = random_action_generator();
    }
  } else if (action == Action::up) {
    action_ = random_action_generator();
    while (action_ == Action::down or
	   action_ == Action::up) {
      action_ = random_action_generator();
    }
  } else if (action == Action::forward) {
    action_ = random_action_generator();
    while (action_ == Action::backward or
	   action_ == Action::forward) {
      action_ = random_action_generator();
    }
  } else if (action == Action::right) {
    action_ = random_action_generator();
    while (action_ == Action::left or
	   action_ == Action::right) {
      action_ = random_action_generator();
    }
  } else if (action == Action::left) {
    action_ = random_action_generator();
    while (action_ == Action::right or
	   action_ == Action::left) {
      action_ = random_action_generator();
    }
  }
  return action_;
}

/*  The condition is to generate an action that not opposed to this
    action. This is more comfortable since the opposed action apply
    high noise on traveled distance. Also this is more logic, since
    allow more variability in the data set */
Quadrotor::Action Quadrotor::
random_action_generator_with_only_opposed_condition(Action action)
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
  }
  return action_;
}

template <class simulator_t>
inline std::ostream& operator<< (std::ostream& out, const Quadrotor::State<simulator_t>& s)
{
  out << s.distances_3D().f1
      <<","
      << s.distances_3D().f2
      <<","
      <<s.height_difference();
  return out;
}
