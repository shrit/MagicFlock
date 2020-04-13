#pragma once

/*  C++ includes */
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include <mlpack/core.hpp>

/* local includes */
#include "logger.hh"

namespace lt = local_types;
using namespace ILMR;

class Math_tools
{

public:
  Math_tools() {}

  template<typename T1, typename T2>
  arma::colvec map_to_arma(std::map<T1, T2>);

  template<typename T>
  T pythagore_leg(T leg, T hypotenuse);

  template<typename T>
  T pythagore_hypotenuse(T leg_1, T leg_2);
};

#include "math_tools.hxx"
