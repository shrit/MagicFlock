# include "perceptron.h"


Perceptron::Perceptron(int input_count, double learning_rate,
		       double threshold)
  :  weights_(input_count),
     learning_rate_(0.1),
     threshold_(0.5)
{
  learning_rate_ = learning_rate;
  threshold_ = threshold;
}


template <typename A,  typename B, typename C>  
void Perceptron::train(A& a, B& b, C& c, unsigned int max_iterations)
{
  if (max_iterations == 0)
    throw std::invalid_argument("The maximum number of iterations cannot be 0.");
  
  unsigned int iterations(0);
  
  for (int i =0; i < max_iterations; i++){
    
    int error_count = 0;

    for(int j = 0; j < a.size(); j++){
      
      bool output = learn(c, a, b);
      
      if (output != c)
	error_count++;
    }
    /*  check this thing very disturbing */
    if (error_count == 0)
      break;
  }
}

template <typename A,  typename B, typename C>  
bool Perceptron::learn(C expected_result, const A& a, const B& b)
{
  bool result = get_result(a, b);
  
  if (result != expected_result) {
    double error = expected_result - result;
        
    for (int i = 0; i < weights_.size(); i++) {
      
      weights_.at(i) += learning_rate_ * error * a.at(i);
    }
  }
  
  return result;
}


template <typename A,  typename B, typename C>  
bool Perceptron::get_result(const A& inputs)
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


