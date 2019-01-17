/*  C++ STL include */

# include <vector>
# include <numeric>
# include <algorithm>
/*  local defined values */

# include "global.hh"

namespace lt = local_types;

class Perceptron
{

public:
  Perceptron(int input_count,
	     double learning_rate,
	     double threshold,
	     const lt::matrix<double>& data_set);
  
  void Classifier(const lt::matrix<double>& m1);
  
private:
  double learning_rate_;
  double threshold_;
  lt::matrix<double> weights_;
  
  
};
