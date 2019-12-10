#pragma once

# include "statictical_predictor.hh"


StatisticalPredictor::StatisticalPredictor(std::string dataset_file,
					   typename std::vector<Quadrotor<simulator_t>>::iterator quad)
  :quad_(quad)
{
  dataset_.read_csv_from_file(dataset_file);
}

void StatisticalPredictor::predict()
{
    
  // Iterate thought out the entire state vectors constructed from the matrix
  std::vector<State> s_t =  dataset_.st_vec();
  std::vector<Actions::Action> a_t =  dataset_.at_vec();
  std::vector<State> s_t_1 =  dataset_.st_1_vec();
  std::vector<tuple<int, int> > dist_ref; 
  
  for (int i = 0; i < s_t.size(); ++i) {
    if (quad_->last_state() == s_t.at(i)) {
      
      if (quad_->current_action() == a_t.at(i)) {
	//return tuple index and distances put them in a vector
	auto dist = mtools_.ecludian_disatnce_states(quad_->last_state(),
						     quad_->current_state(),
						     s_t, s_t_1, 0);
	
	dist_ref.push_back(dist);
      } else  {
	auto dist = mtools_.ecludian_disatnce_states(quad_->last_state(),
						     quad_->current_state(),
						     s_t, s_t_1, 1);

	dist_ref.push_back(dist);
	
      }
      /*  Print using spdlog the values of the value of the iterator  */
      // do argmin on the distances.
      auto it = std::min_element(dist_ref.begin(), dist_ref.end());
      
	

      
      // do argmin on st+2. get the best action at+1
    }    
  }    
}

Actions::Action StatisticalPredictor::get_predicted_action()
{
  Actions::Action predicted_follower_action;

  return predicted_follower_action;    
}
