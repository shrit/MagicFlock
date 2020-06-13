#pragma once

template<typename type>
class ExoMovingAverage
{
public:
  ExoMovingAverage(const type& alpha, const type& initial_value = type());

  type input(const type& new_value);
  const type& output() const;

  void reset();

private:
  const type alpha_;         //< Smoothing coefficient.
  const type initial_value_; //< Initial value.
  type value_;               //< Stored value.
};

#include "exponential_moving_average.hxx"
