# include "quadcopter.hh"

template <class simulator_t>
Quadcopter<simulator_t>::Quadcopter()
  :distribution_(0.0, 1.0),
   distribution_int_(0, 3),
   generator_(random_dev())
  
{}

template <class simulator_t>
Quadcopter<simulator_t>::State::State(std::shared_ptr<simulator_t> sim_interface):
  sim_interface_(std::move(sim_interface))   
{  
  rssi_ = sim_interface_->rssi();  
  height_ = sim_interface_->positions().f2.z;  
  z_orinetation_  = sim_interface_->orientations().f2.z;
  dists_ =   mtools_.triangle_side(sim_interface_->positions());  
}


template <class simulator_t>
lt::rssi<double> Quadcopter<simulator_t>::State::
signal_strength () const
{
  return rssi_ ; 
}

template <class simulator_t>
double Quadcopter<simulator_t>::State::
height () const
{/*  need to make it according to what quad?? */
  return height_;
}

template <class simulator_t>
double Quadcopter<simulator_t>::State::
orientation () const
{
  return z_orinetation_;
}

template <class simulator_t>
lt::triangle<double> Quadcopter<simulator_t>::State::
distances () const
{
  return dists_;
}

template <class simulator_t>
state_printer Quadcopter<simulator_t>::State::
create_printer_struct(Quadcopter<simulator_t>::State state)
{
  state_printer sp;
  sp.rssi = state.signal_strength();
  sp.height = state.height();
  sp.distances = state.distances();
  sp.orientation = state.orientation();
  return sp;
}

template <class simulator_t>
typename Quadcopter<simulator_t>::Reward Quadcopter<simulator_t>::
action_evaluator(lt::triangle<double> old_dist,
		 lt::triangle<double> new_dist)
{
  LogInfo() << "F1 differences: " << std::fabs(old_dist.f1 - new_dist.f1);
  LogInfo() << "F2 differences: " << std::fabs(old_dist.f2 - new_dist.f2);
  
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);
  
   Reward reward = Reward::very_bad;
  
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

template <class simulator_t>
std::vector<typename Quadcopter<simulator_t>::Action> Quadcopter<simulator_t>::
possible_actions() const
{ return possible_actions_; }

template <class simulator_t>
typename Quadcopter<simulator_t>::Action Quadcopter<simulator_t>::
randomize_action()
{  
  int random_action = distribution_int_(generator_);
  
  LogInfo() << "Random action value: " << random_action ;

  Action action = Action::forward ; 
  
  if( random_action == 0){
    action = Action::forward ;
  }
  else if(random_action == 1){
    action = Action::backward ;
  }    
  else if(random_action == 2){
    action = Action::left ;
  }
  else if (random_action == 3){
    action = Action::right ;   
  }
   return action;
}
