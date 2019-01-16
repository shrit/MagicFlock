# include "perceptron.h"


Perceptron::Perceptron(int input_count, const double learning_rate = 0.1,
		       const double threshold = 0.5)
  : weights_(input_count)
{
  learning_rate_ = learning_rate;
  threshold_ = threshold;
}



void Perceptron::train(std::vector<TrainingItem> & training_set, unsigned int max_iterations)
{
  if (max_iterations == 0)
    throw std::invalid_argument("The maximum number of iterations cannot be 0.");
  
  unsigned int iterations(0);
  
  for (int i =0; i < line_number_; i++){
    
    int error_count = 0;

    for(int j = 0; j < vector.length(); j++){
      
      bool output = learn(.output(), .inputs());
      
      
    }
    
    
    
    for (auto &item : training_set) {
      bool output = learn(item.output(), item.inputs());
      
      if (output != item.output())
	error_count++;
    }
    /*  check this thing very disturbing */
    if (error_count == 0)
      break;
  }
}

bool Perceptron::learn(bool expected_result, const std::vector<double> &inputs)
{
  bool result = get_result(inputs);
  if (result != expected_result) {
    // Convert boolean to a number
    double error = (expected_result ? 1 : 0) - (result ? 1 : 0);
    for (int i = 0; i < weights_.size(); i++) {
      weights_[i] += learning_rate_ * error * inputs[i];
    }
  }
  
  return result;
}

bool Perceptron::get_result(const std::vector<double> &inputs)
{
  if (inputs.size() != weights_.size())
    throw std::invalid_argument("Invalid number of inputs. Expected: "
				+ weights_.size());
  
    return dot_product(inputs, weights_) > threshold_;
}

double Perceptron::dot_product(const std::vector<double> &v1,
			       const std::vector<double> &v2)
{
      return std::inner_product(v1.begin(), v1.end(), v2.begin(), 0);
}


