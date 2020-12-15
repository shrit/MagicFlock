#pragma once

#include <IL4MRC/util/logger.hpp>
#include <mlpack/prereqs.hpp>

class OneHotEncoding
{
public:
  OneHotEncoding() {}

  template<typename Arg, typename Arg2>
  arma::colvec to_one_hot_encoding(Arg arg, Arg2 number_of_class);

  template<typename Arg>
  arma::colvec to_one_hot_encoding(ignition::math::Vector3d vec,
                                   Arg number_of_class);

  template<typename Arg>
  arma::uword from_one_hot_encoding(arma::Col<Arg> col_vec);
};

#include "one_hot_encoding_impl.hpp"
