#pragma once

#include <mlpack/core.hpp>

template<class InputType = arma::mat, class OutputType = arma::uword>
class Argmin
{
public:
  Argmin(InputType mat, InputType mat2);
  Argmin(InputType mat, InputType mat2, arma::uword remove_col);

  OutputType best_action() const;
  OutputType best_index() const;
  
private:
  OutputType action_result_ = 0;
  OutputType index_result_ = 0;
};

#include "argmin.hxx"
