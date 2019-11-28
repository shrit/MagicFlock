#pragma once

# include "quadrotor.hh"
template <class simulator_t>
Quadrotor<simulator_t>::Quadrotor(unsigned int id,
				  std::string name,
				  std::shared_ptr<simulator_t> sim_interface)
  :id_(id),
   name_(name),
   sim_interface_(sim_interface)
{
  data_set_.init_dataset_directory();
}

template <class simulator_t>
unsigned int Quadrotor<simulator_t>::id() const
{ return id_; }

template <class simulator_t>
std::string Quadrotor<simulator_t>::name() const
{ return name_; }

template <class simulator_t>
void Quadrotor<simulator_t>::add_nearest_neighbor_id(unsigned int id)
{ nearest_neighbors_.push_back(id); }

template <class simulator_t>
std::vector<unsigned int> Quadrotor<simulator_t>::nearest_neighbors() const
{ return nearest_neighbors_; }

template <class simulator_t>
void Quadrotor<simulator_t>::sample_state()
{
  State<simulator_t> state(sim_interface_, id_, nearest_neighbors_);
  current_state_ = state;
  all_states_.push_back(state);
}

template <class simulator_t>
State<simulator_t> Quadrotor<simulator_t>::current_state() const
{ return current_state_; }

template <class simulator_t>
std::vector<State<simulator_t>> Quadrotor<simulator_t>::all_states() const
{ return all_states_; }

template <class simulator_t>
State<simulator_t> Quadrotor<simulator_t>::last_state()  
{
  if (all_states_.size() > 1) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 1);
    last_state_ = (*it_state);
  }  
  return last_state_; 
}

template <class simulator_t>
State<simulator_t> Quadrotor<simulator_t>::before_last_state()
{
  if (all_states_.size() > 2) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 2);
    before_last_state_ = (*it_state);
  }  
  return before_last_state_;  
}

template <class simulator_t>
void Quadrotor<simulator_t>::reset_all_states()
{ all_states_.clear(); }

template <class simulator_t>
void Quadrotor<simulator_t>::current_action(Actions::Action action)
{
  all_actions_.push_back(action);
  current_action_ = action;
}

template <class simulator_t>
Actions::Action Quadrotor<simulator_t>::current_action() const
{ return current_action_; } 

template <class simulator_t>
Actions::Action Quadrotor<simulator_t>::last_action()
{
  if (all_actions_.size() > 1) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 1);
    last_action_ = (*it_action);
  }
  return last_action_;
}

template <class simulator_t>
std::vector<Actions::Action> Quadrotor<simulator_t>::all_actions() const
{ return all_actions_; }
  
template <class simulator_t>
void Quadrotor<simulator_t>::reset_all_actions()
{ all_actions_.clear(); }

template <class simulator_t>
void Quadrotor<simulator_t>::register_data_set()
{
  data_set_.save_csv_data_set(before_last_state(),				      
			      mtools_.to_one_hot_encoding(last_action(), 7),
			      last_state(),
			      mtools_.to_one_hot_encoding(current_action(), 7),
			      current_state()	      
			      );  
}

template <class simulator_t>
void Quadrotor<simulator_t>::register_histogram(int count)
{
  /*  Save a version of the time steps to create a histogram */
  /*  Increase one since count start from 0 */
  /*  Need to find a solution for this with a timer */
  count++;
  data_set_.save_controller_count(count);
  mtools_.histogram(count);
  data_set_.save_histogram(mtools_.get_histogram<int>());
}

template <class simulator_t>
bool Quadrotor<simulator_t>::examin_geometric_shape()
{
  bool shape = false;
  if (mtools_.is_good_shape(id_,
			    nearest_neighbors_,
			    sim_interface_->positions()) == true) {
    shape = true;
  }
  return shape;
}