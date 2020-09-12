#pragma once

/*  C++ includes */
#include <algorithm>
#include <map>
#include <utility>
#include <vector>

#include <mlpack/core.hpp>

/* local includes */
#include "logger.hpp"

using namespace ILMR;

class ArmaHelper
{

public:
  ArmaHelper() {}

  template<typename T1, typename T2>
  arma::colvec map_to_arma(std::map<T1, T2>);
  
  template<typename T>
  arma::colvec vec_to_arma(std::vector<T> vec); 
};

#include "arma_helper_impl.hpp"
