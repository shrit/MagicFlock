# include "quadcopter.hh"


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
