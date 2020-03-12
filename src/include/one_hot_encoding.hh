#pragma once

class OneHotEncoding
{
public:
  OneHotEncoding() {}

  template<typename Arg, typename Arg2>
  arma::colvec to_one_hot_encoding(Arg arg, Arg2 number_of_class);

  template<typename Arg>
  arma::uword from_one_hot_encoding(arma::Col<Arg> col_vec);
};

#include "one_hot_encoding.hxx"
