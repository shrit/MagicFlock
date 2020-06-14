#pragma once

template<typename type>
ExpoMovingAverage<type>::ExpoMovingAverage()
{
  // Nothing to do here
}

template<typename type>
ExpoMovingAverage<type>::ExpoMovingAverage(const type& alpha,
                                           const type& initial_value)
  : alpha_(alpha)
  , initial_value_(initial_value)
  , value_(initial_value)
{
  assert(alpha > 0 and alpha <= 1);
}

template<typename type>
type
ExpoMovingAverage<type>::input(const type& new_value)
{
  return value_ = alpha_ * new_value + (1 - alpha_) * value_;
}

template<typename type>
type
ExpoMovingAverage<type>::output() const
{
  return value_;
}

template<typename type>
void
ExpoMovingAverage<type>::reset()
{
  value_ = initial_value_;
}
