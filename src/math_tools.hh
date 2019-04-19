#ifndef MATH_TOOLS_HH
#define MATH_TOOLS_HH

# include <algorithm>
# include <cmath>
# include <numeric>
# include <vector>
# include <utility>

# include "log.hh"

class Math_tools {


public:

  Math_tools();

  template <typename Arg, typename... Args>
  Arg cantor_pairing(Arg arg, Args... args);

  double gaussian_noise(std::vector<lt::triangle<double>> ideal_dist,
			std::vector<double> drift_f3);
  
  bool is_triangle(lt::triangle<double> t);
  
  lt::triangle<double> triangle_side(lt::positions<double> pos);
  
  template <typename Arg>
  Arg variance(std::vector<Arg> vec);
    
private:

  std::vector<float> lower_threshold_;
  std::vector<float> upper_threshold_;
  
};

# include "math_tools.hxx"

#endif
