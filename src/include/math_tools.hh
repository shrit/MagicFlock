#pragma once

/*  C++ includes */
# include <algorithm>
# include <cmath>
# include <numeric>
# include <type_traits>
# include <vector>
# include <utility>

# include <armadillo>

/* local includes */
# include "global.hh"
# include "logger.hh"

namespace lt = local_types;
using namespace ILMR;

class Math_tools {

public:

  Math_tools() {}

  template <typename T>
  double deformation_error_one_follower(lt::triangle<T> old_dist,
					lt::triangle<T> new_dist);

	template <typename T>
  std::vector<double> distances_to_neighbors(unsigned int id,
					     std::vector<unsigned int> nearest_neighbors,
					     std::vector<lt::position3D<T>> positions);

	template <typename T>
  double distance_a_2_b(std::vector<lt::position3D<T>> positions,
			unsigned int id_a,
			unsigned int id_b);

  template <typename T>
  double gaussian_noise(std::vector<lt::triangle<T>> ideal_dist,
			std::vector<T> drift_f3);
  template <typename T>
  long long unsigned int
  index_of_max_value(const std::vector<T>& vec);

  template <typename T>
  long long unsigned int
  index_of_min_value(const std::vector<T>& vec);

	template <typename T>
  bool is_good_shape(unsigned int id,
		     std::vector<unsigned int> nearest_neighbors,
		     std::vector<lt::position3D<T>> positions);

  template <typename T>
  std::map<T, T> get_histogram();

  template <typename T>
  void histogram(T times);

  template <typename Arg>
  Arg mean(std::vector<Arg> vec);

  template <typename Arg, typename Arg2>
  std::vector<bool> to_one_hot_encoding(Arg arg,
																				Arg2 number_of_class);
  
  template <typename Arg>
  int from_one_hot_encoding(std::vector<Arg> values);

  template <typename Arg>
  std::vector<double> to_std_vector(Arg arg);

  template <typename Arg>
  bool hamming_distance_one_hot(std::vector<Arg> v1, std::vector<Arg> v2);

	template <typename T>
  int ecludian_distance(T d1, T d2);

  template <typename T>
  double traveled_distances(lt::position3D<T> pos_t, 
  lt::position3D<T> pos_t_1);

  template <typename T>
  T pythagore_leg(T leg, T hypotenuse);

  template <typename T>
  T pythagore_hypotenuse(T leg_1, T leg_2);

  template <typename Arg>
  Arg variance(std::vector<Arg> vec);

private:
  std::map <int, int> histo_;
  double lower_threshold_ = 1; /*  1 meters minimum distance between quadrotors */
  double upper_threshold_ = 5; /*  5 meters maximum distance between quadrotors */
};

# include "math_tools.hxx"
