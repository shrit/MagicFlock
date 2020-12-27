#pragma once

#include <algorithm>
#include <map>
#include <numeric>
#include <type_traits>
#include <vector>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector4.hh>
#include <mlpack/prereqs.hpp>

#include <MagicFlock/util/logger.hpp>

using namespace ILMR;

class VectorHelper
{
public:
  VectorHelper() {}

  template<typename Arg>
  Arg variance(std::vector<Arg> vec);

  template<typename Arg>
  Arg mean(std::vector<Arg> vec);

  template<typename Arg>
  std::vector<int> fill_range(Arg value);

  template<typename T>
  long long unsigned int index_of_max_value(const std::vector<T>& vec);

  template<typename T>
  long long unsigned int index_of_min_value(const std::vector<T>& vec);

  template<typename Arg>
  std::vector<double> to_std_vector(Arg arg);

  template<typename Arg>
  std::vector<double> ignition_2_std_vector(Arg arg);

  template<typename T1, typename T2>
  std::vector<T2> map_to_vector(std::map<T1, T2>);
};

#include "Vector_impl.hpp"
