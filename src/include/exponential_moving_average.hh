#pragma once

#include "logger.hh"

template<typename type>
class ExpoMovingAverage
{
public:
  ExpoMovingAverage();
  ExpoMovingAverage(double alpha);
  ExpoMovingAverage(double alpha, const type& initial_value);

  type input(const type& new_value);
  const type& output() const;
  const double& alpha() const;

  type initial_value() const;
  type& initial_value();

  void reset();

private:
  // Keep alpha const, as we do not need to changed now
  const double alpha_{ 0.9 };  //! Smoothing coefficient.
  type initial_value_; //! Initial value.
  type value_;               //! Stored value.
};

#include "exponential_moving_average.hxx"
