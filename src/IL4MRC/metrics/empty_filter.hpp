#pragma once

#include <IL4MRC/util/logger.hpp>

template<typename type>
class EmptyFilter
{
public:
  EmptyFilter();
  EmptyFilter(double alpha);
  EmptyFilter(double alpha, const type& initial_value);

  type input(const type& new_value);
  const type& output() const;
  const double& alpha() const;

  type initial_value() const;
  type& initial_value();

  void reset();

private:

  const double alpha_{ 0.9 };  //! Smoothing coefficient.
  type initial_value_; //! Initial value.
  type value_;               //! Stored value.
};

#include "exponential_moving_average_impl.hpp"
