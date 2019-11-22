#include "action.hh"

Actions::Action()
  :distribution_int_(0, 5),
   generator_(random_dev())
{}

std::string Actions::action_to_str(Action action)
{
  std::string string_action = "";
  switch (action) {
  case forward:
    string_action = "Forward";
    break;
  case backward:
   string_action =  "Backward"; 
    break;
  case left:
    string_action =  "Left";
    break;
  case right:
    string_action =  "right";
    break;
  case up:
    string_action =  "Up";
    break;
  case down:
    string_action =  "Down";
    break;
  case NoMove:
    string_action =  "NoMove";
    break;
  case Unknown:
    string_action =  "Unknown";
    break;    
  }
  return string_action;    
}

/* Get the best action from the model according to the best values */
Actions::Action Actions::
extract_action_from_index(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  Action::Action action = Action::Action::Unknown;
  /*  Access matrix values according to a given index  */
  /*  Only one action exist that equal 1 in each row of 
   a matrix */  
  if (features(index, 14) == 1) {
    action = Action::Action::forward;
  } else if (features(index, 15) == 1) {
    action = Action::Action::backward;
  } else if (features(index, 16) == 1) {
    action = Action::Action::left;
  } else if (features(index, 17) == 1) {
    action = Action::Action::right;
  } else if (features(index, 18) == 1) {
    action = Action::Action::up;
  } else if (features(index, 19) == 1) {
    action = Action::Action::down;
  } else if (features(index, 20) == 1) {
    action = Action::Action::NoMove;
  }  
  return action;
}

Actions::Action Actions::
int_to_action(int action_value)
{
  Action action;
  return action = static_cast<Action>(action_value);  
}

std::vector<Actions::Action> Actions::
all_possible_actions() const
{ return possible_actions_; }

Actions::Action Actions::
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
Actions::Action Actions::
random_action_generator_with_all_conditions(Action action)
{
  Action action_ = Action::NoMove;
  
  if (action == Action::backward) {
    action_ = random_action_generator();
    while (action_ == Action::forward or
	   action_ == Action::backward) {
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
Actions::Action Actions::
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
