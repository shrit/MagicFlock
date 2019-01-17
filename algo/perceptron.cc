# include "perceptron.h"


Perceptron::Perceptron(int input_count, double learning_rate,
		       double threshold, const lt::matrix<double>& data_set)
  :  weights_(input_count, std::vector<double>(4,0.0)),
     learning_rate_(0.1),
     threshold_(0.2)
{
  learning_rate_ = learning_rate;
  threshold_ = threshold;


  Classifier(data_set);
  
}


void Perceptron::Classifier(const lt::matrix<double>& data_set)
{
  /*  Extracting error from the data set, and save it into 
   * A vector  
   *
   */
  std::vector<double> error;              
  for(int  i = 0; i < data_set.size(); i++){
    double sum_of_elems = 0;
    std::for_each(data_set.at(i).begin()+5, data_set.at(i).end(), [&] (double n) {
							sum_of_elems += n;
						      });
    error.push_back(sum_of_elems);
  }

  std::cout << "error of the data set" << error <<std::endl;

  // lt::matrix<double> data{data_set.size(), std::vector<double>(4)};
  
  
  // for(int  i = 0; i < data_set.size(); i++){					    
  										    
  //   std::copy_n(data_set.at(i).begin(), data_set.at(i).begin() + 4, data.begin()); 
  // }										    
  										    
  // std::cout << "the data: " << data <<std::endl;				    
  

  
  int error_count = 0;


  // /*  Trainning loop used to adjust the weights of the input, in order to 
  //  be equal to the ouput. */
  // /*  1 we start by calculate the weights  */
  // /* 2 we compare the result obtained with the known error */
  
  for(;;){        
    
    std::vector<double> result;
    for(int  i = 0; i < data_set.size(); i++){
      //     /*  Here we filter only the good data_set from the bad one */
      //     //to check wether it is treating the error or not, printing needed
      double value =
	std::inner_product(data_set.at(i).begin(), data_set.at(i).begin()+4, weights_.at(i).begin(), 0.0);

      /*  the following is throwing exceptions because it is empty as the data set 
       will never be smaller than the threshold*/

      result.push_back(value);
      
      if (value > threshold_){
	
	std::vector<double> error_diff;
	for(int j =0; j < result.size(); j++){
	  
	  if (result.at(j) != error.at(j)) {
	
    	error_diff.push_back(error.at(j) - result.at(j));
    	error_count++;
      }   
	}
    
	std::cout << "error diff: " << error_diff << std::endl;
    
    for (int i = 0; i < weights_.size(); i++) {
      for(int j =0; j < 4; j ++)
    	weights_.at(i).at(j) += learning_rate_ * error_diff.at(i) * data_set.at(i).at(j);    
    }
    


	
	
      }
     }
    
    // std::cout << "result " << result << std::endl;
    
    
    // // Compare the result obtained with the one calculated
     
    std::cout << "Weights: " <<  weights_ << std::endl;
    //   if (error_count == 0)
    //  break;
    
  }  

}

