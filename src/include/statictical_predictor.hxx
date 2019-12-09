#pragma once

# include "statictical_predictor.hh"


StatisticalPredictor::StatisticalPredictor(std::string dataset_file,
					   typename std::vector<Quadrotor<simulator_t>>::iterator quad)
{
  dataset.load(dataset_file, csv_ascii);  
}

void StatisticalPredictor::predict()
{

  // We need to create a state from the first 3 column from a matrix
  // It is much better to parse data set line by line into state, actions, nextstate , etc
  /*  Then all the parsed state and actions can be agglomerated into matries.
   Then it should be easy to read each line of these matrices since they are seperated */
  std::vector<double> state = dataset.each_row([](){
						 std::vector<double> vec;
						 arma::colvec
						 return vec;
					       });
  

  StateConstructor(/*  Armadillo row state matrix */);
  ActionConstructor(/*  Armadillo row state matrix */)
    
  // Iterate thought out the entire state vectors constructed from the matrix
  
  for(; it != it_end; ++it) {
    
    if (quad.current_state() == it.current_state()) {
      // Get actions and s t+1 from the data set,
      // do hamming and eculdian distance test
      // do argmin on the distances.
      // do argmin on st+2. get the best action at+1
    }    
  }

  
  
}

Actions::Action StatisticalPredictor::get_predicted_action()
{
  Actions::Action predicted_follower_action;

  return predicted_follower_action;    
}
