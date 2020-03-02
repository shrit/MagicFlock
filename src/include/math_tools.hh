#pragma once

/*  C++ includes */
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

#include <mlpack/core.hpp>

/* local includes */
#include "global.hh"
#include "logger.hh"

namespace lt = local_types;
using namespace ILMR;

class Math_tools
{

public:
  Math_tools() {}

  template<typename T>
  std::map<unsigned int, double> distances_to_neighbors(
    unsigned int id,
    std::vector<unsigned int> nearest_neighbors,
    std::vector<lt::position3D<T>> positions);

  template<typename T>
  double distance_a_2_b(std::vector<lt::position3D<T>> positions,
                        unsigned int id_a,
                        unsigned int id_b);

  template<typename T>
  bool is_good_shape(unsigned int id,
                     std::vector<unsigned int> nearest_neighbors,
                     std::vector<lt::position3D<T>> positions);

  template<typename T>
  std::map<T, T> get_histogram();

  template<typename T>
  void histogram(T times);

  template<typename KeyType, typename ValueType>
  std::pair<KeyType, ValueType> get_max_histogram(
    const std::map<KeyType, ValueType>& x);

  template<typename Arg, typename Arg2>
  arma::colvec to_one_hot_encoding(Arg arg, Arg2 number_of_class);

  template<typename Arg>
  arma::uword from_one_hot_encoding(arma::Col<Arg> col_vec);

  template<typename Arg>
  std::vector<double> to_std_vector(Arg arg);

  template<typename T1, typename T2>
  std::vector<T2> map_to_vector(std::map<T1, T2>);

  template<typename T1, typename T2>
  arma::colvec map_to_arma(std::map<T1, T2>);

  template<typename Arg>
  bool hamming_distance_one_hot(std::vector<Arg> v1, std::vector<Arg> v2);

  template<typename T>
  int ecludian_distance(T d1, T d2);

  template<typename T>
  double traveled_distances(lt::position3D<T> pos_t, lt::position3D<T> pos_t_1);

  template<typename T>
  T pythagore_leg(T leg, T hypotenuse);

  template<typename T>
  T pythagore_hypotenuse(T leg_1, T leg_2);

private:
  std::map<int, int> histo_;
  double lower_threshold_ =
    1; /*  1 meters minimum distance between quadrotors */
  double upper_threshold_ =
    8; /*  8 meters maximum distance between quadrotors */
};

#include "math_tools.hxx"
