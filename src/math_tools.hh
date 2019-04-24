#pragma once

# include <algorithm>
# include <cmath>
# include <numeric>
# include <type_traits>
# include <vector>
# include <utility>

# include "log.hh"
# include "global.hh"

namespace lt = local_types;

class Math_tools {

public:

  Math_tools();
  
  double gaussian_noise(std::vector<lt::triangle<double>> ideal_dist,
			std::vector<double> drift_f3);
  
  bool is_triangle(lt::triangle<double> t);
  
  lt::triangle<double> triangle_side(lt::positions<double> pos);

  template <typename Arg, typename Arg2>
  std::vector<bool> to_one_hot_encoding(Arg arg,
			  Arg2 number_of_class);
  
  template <typename Arg>
  Arg variance(std::vector<Arg> vec);
    
private:

  std::vector<float> lower_threshold_;
  std::vector<float> upper_threshold_;
  
};

# include "math_tools.hxx"
