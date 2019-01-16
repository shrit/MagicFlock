# include <vector>




class Perceptron
{

public:
  Perceptron(int input_count, const double learning_rate = 0.1,
	     const double threshold = 0.5);
  
  
  void train(std::vector<>& , unsigned int max_iterations);  
  bool learn(bool expected_result, const std::vector<double> &inputs);
  bool Perceptron::get_result(const std::vector<double> &inputs);
  double Perceptron::dot_product(const std::vector<double> &v1,
				 const std::vector<double> &v2);
  
private:
  double learning_rate_;
  double threshold_;
  std::vector<double> weights_;
  
  
}
