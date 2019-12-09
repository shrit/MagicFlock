#pragma once

# include "statictical_predictor.hh"


StatisticalPredictor::StatisticalPredictor(std::string dataset_file,
					   typename std::vector<Quadrotor<simulator_t>>::iterator quad)
{
  dataset.load(dataset_file, csv_ascii);  
}

void StatisticalPredictor::predict()
{
  // Iterate throughout only the first column
  /*  Other columns are compared later, if the first one is Ok */
  arma::mat::iterator it     = dataset.begin();
  arma::mat::iterator it_end = dataset.end();

  // We need to create a state from the first 3 column from a matrix


  // Iterate thought out the entire state vectors constructed from the matrix
  
  for(; it != it_end; ++it) {
    
    if (quad.current_state() == it.current_state()) {
      // Get actions and s t+1 from the data set,
      // do hamming and eculdian test
      // do argmin
    }    
  }

  
  
}

Actions::Action StatisticalPredictor::get_predicted_action()
{
  Actions::Action predicted_follower_action;

  return predicted_follower_action;    
}
