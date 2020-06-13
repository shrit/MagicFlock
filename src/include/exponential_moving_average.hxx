#pragma once

template<typename T>
ExoMovingAverage<T>::ExoMovingAverage(const type& alpha,
                                      const type& initial_value)
  : alpha_(alpha)
  , initial_value_(initial_value)
  , value_(initial_value)
{
  assert(alpha > 0 and alpha <= 1);
}

template<typename T>
typename ExoMovingAverage<T>::type
ExoMovingAverage<T>::input(const type& new_value)
{
  return value_ = alpha_ * new_value + (1 - alpha_) * value_;
}

template<typename T>
typename ExoMovingAverage<T>::type const&
ExoMovingAverage<T>::output() const
{
  return value_;
}

template<typename T>
void
ExoMovingAverage<T>::reset()
{
  value_ = initial_value_;
}
