#pragma once

# include "quadcopter.hh"
template <class simulator_t>
Quadrotor<simulator_t>::Quadrotor(unsigned int id,
				  std::string name)
  :id_(id),
   name_(name)
{}

template <class simulator_t>
unsigned int Quadrotor<simulator_t>::id() const
{ return id_; }

template <class simulator_t>
std::string Quadrotor<simulator_t>::name() const
{ return name_; }

template <class simulator_t>
State<simulator_t> Quadrotor<simulator_t>::current_state() const
{
  State<simulator_t> state;

  /*  Use the neighbor list in order to call the state directly */
  if (id_ == 0 or name_ == "leader") {
    /*  Generate state according to leader necessity */
    
  } else if (id_ == 1 or name_ == "follower_1") {
    /*  Generate state according to the first follower necessity */
  } else if (id_ == 2 or name_ == "follower_2") {
    /*  Generate state according to the second follower necessity */
  }
  all_states_.push_back(state);
  return state;
}

template <class simulator_t>
void Quadrotor<simulator_t>::add_nearest_neighbor_id(unsigned int id)
{
  nearest_neighbors_.push_back(id);
}

template <class simulator_t>
std::vector<unsigned int> Quadrotor<simulator_t>::nearest_neighbors() const
{ return nearest_neighbors_; }
  
template <class simulator_t>
std::vector<State<simulator_t>> Quadrotor<simulator_t>::all_states() const
{ return all_states_; }

template <class simulator_t>
State<simulator_t> Quadrotor<simulator_t>::last_state()
{  return last_state_; // As last_action the same things to do }

template <class simulator_t>
void Quadrotor<simulator_t>::reset_all_states()
{ all_states_.clear(); }

template <class simulator_t>
Actions::Action Quadrotor<simulator_t>::current_action()
{ all_actions_.push_back(current_action_);
  return current_action_;
}

template <class simulator_t>
Actions::Action Quadrotor<simulator_t>::last_action() const
{/*  Use the vector r begin +1
  be sure that the vector has more than 1 element*/
  return last_action_; }

template <class simulator_t>
std::vector<Actions::Action> Quadrotor<simulator_t>::all_actions() const
{ return all_actions_; }
  
template <class simulator_t>
void Quadrotor<simulator_t>::reset_all_action()
{ all_actions_.clear(); }
