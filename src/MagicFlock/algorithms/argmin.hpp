#pragma once

#include <mlpack/core.hpp>

#include <MagicFlock/util/logger.hpp>

template<class InputType = arma::mat, class OutputType = arma::uword>
class Argmin
{
public:
  Argmin(InputType mat, InputType mat2);
  Argmin(InputType mat, InputType mat2, arma::uword remove_col);

  OutputType min_index() const;
  
private:
  OutputType min_index_ = 0;
};

#include "argmin_impl.hpp"
