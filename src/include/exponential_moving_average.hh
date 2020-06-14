#pragma once

template<typename type>
class ExpoMovingAverage
{
public:
  ExpoMovingAverage();

  ExpoMovingAverage(const type& alpha, const type& initial_value = type());

  type input(const type& new_value);
  type output() const;

  void reset();

private:
  const type alpha_ {0.9};         //< Smoothing coefficient.
  const type initial_value_; //< Initial value.
  type value_;               //< Stored value.
};

#include "exponential_moving_average.hxx"
