#pragma once

#include <mlpack/core.hpp>

template<class InputType = arma::mat, class OutputType = arma::uword>
class Argmin
{
public:
    Argmin(InputType mat, InputType mat2);
    OutputType result() const;
    
private:
  OutputType argmin_result_ = 0;
};

#include "argmin.hxx"
