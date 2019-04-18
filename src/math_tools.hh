#ifndef MATH_TOOLS_HH
#define MATH_TOOLS_HH

# include <algorithm>
# include <cmath>
# include <numeric>
# include <vector>

# include "log.hh"

class Math_tools {


public:

  Math_tools();

  template <typename Arg, typename... Args>
  Arg cantor_pairing(Arg&& arg, Args&&... args);

  double gaussian_noise(std::vector<lt::triangle<double>> ideal_dist);
  
  bool is_triangle(lt::triangle<double> t);
  
  lt::triangle<double> triangle_side(lt::positions<double> pos);
  
  double variance(double mean);
  
private:



  
};


# include "math_tools.hxx"

#endif
