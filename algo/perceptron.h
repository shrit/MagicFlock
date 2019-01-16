/*  C++ STL include */

# include <vector>
# include <numeric>
/*  local defined values */

# include "global.hh"

namespace lt = local_types;

class Perceptron
{

public:
  Perceptron(int input_count, double learning_rate,
	     double threshold);
  
  template <typename A,  typename B, typename C>  
  void train(A& a, B& b, C& c, unsigned int max_iterations);

  template <typename A,  typename B, typename C>
  bool learn(C  expected_result, const A& a, const B& b);
  
  template <typename A,  typename B, typename C>    
  bool get_result(const A& inputs);

  
  double dot_product(const std::vector<double> &v1,
				 const std::vector<double> &v2);
  
private:
  double learning_rate_;
  double threshold_;
  std::vector<double> weights_;
  
  
};
