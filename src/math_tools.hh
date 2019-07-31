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

  Math_tools() :
    lower_threshold_{4, 4, 4},
    upper_threshold_{9, 9, 9}
  {}

  template <typename T>
  double deformation_error_one_follower(lt::triangle<T> old_dist,
					lt::triangle<T> new_dist);
  
  template <typename T>
  double gaussian_noise(std::vector<lt::triangle<T>> ideal_dist,
			std::vector<T> drift_f3);
  template <typename T>
  long long unsigned int
  index_of_highest_value(const std::vector<T>& vec);
  
  template <typename T>
  bool is_triangle(lt::triangle<T> t);

  template <typename T>
  std::vector<int> histogram(std::vector<T> vec);
  
  template <typename T>
  lt::triangle<double> triangle_side(lt::positions<T> pos);

  template <typename Arg, typename Arg2>
  std::vector<bool> to_one_hot_encoding(Arg arg,
			  Arg2 number_of_class);

  template <typename T>
  T pythagore_leg(T leg, T hypotenuse);

  template <typename T>
  T pythagore_hypotenuse(T leg_1, T leg_2);
  
  template <typename Arg>
  Arg variance(std::vector<Arg> vec);
    
private:

  std::vector<float> lower_threshold_;
  std::vector<float> upper_threshold_;
  
};

# include "math_tools.hxx"
