#pragma once

# include "statistical_predictor.hh"

template<class simulator_t>
StatisticalPredictor<simulator_t>::
StatisticalPredictor(std::string dataset_file,
		     typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  :quad_(quad)
{
  dataset_.read_dataset_file(dataset_file);
}

template<class simulator_t>
void StatisticalPredictor<simulator_t>::predict()
{   
  std::vector<State<simulator_t>> s_t =  dataset_.st_vec();
  std::vector<Actions::Action> a_t =  dataset_.at_vec();
  std::vector<State<simulator_t>> s_t_1 =  dataset_.st_1_vec();
  std::vector<Actions::Action> a_t_1 =  dataset_.at_1_vec();
  std::vector<std::tuple<double, int>> dist_ref; 
  
  for (int i = 0; i < s_t.size(); ++i) {
    if (quad_->last_state() == s_t.at(i)) {
      
      if (quad_->current_action() == a_t.at(i)) {

        StateDistance<simulator_t> state_distance(quad_->last_state(),
						  quad_->current_state(),
						  s_t, s_t_1);
	
	double distance = state_distance.distance();	
	dist_ref.push_back(std::make_tuple(distance, i));
	
      } else {
	StateDistance<simulator_t> state_distance(quad_->last_state(),
						  quad_->current_state(),
						  s_t, s_t_1);
	
	double distance = state_distance.distance();
	distance ++; // Adding one since the actions are not the same
	dist_ref.push_back(std::make_tuple(distance, i));       
      }
    }
  }
  
  /*  Print using spdlog the values of the value of the iterator  */
  // do argmin on the distances.
  auto it = std::min_element(dist_ref.begin(), dist_ref.end());
  int index = 0;
  std::tie(std::ignore, index) = (*it);
  // do argmin on st+2. get the best action at+1
  predicted_follower_action_ = a_t_1.at(index);  
}

template<class simulator_t>
Actions::Action StatisticalPredictor<simulator_t>
::get_predicted_action() const
{ return predicted_follower_action_; }
