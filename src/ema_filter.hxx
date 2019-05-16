#pragma once

# include "ema_filter.hh"
 

template <typename T>
EWMAFilter<T>::EWMAFilter(const value_type&	alpha,
			  const value_type&	initial_value)
  : alpha_(alpha),
    initial_value_(initial_value),
    value_(initial_value)
{ assert(alpha > 0 and alpha <= 1); }

template <typename T>
typename EWMAFilter<T>::value_type
EWMAFilter<T>::input(const value_type& new_value)
{ return value_ = alpha_ * new_value + (1 - alpha_) * value_; }

template <typename T>
typename EWMAFilter<T>::value_type const&
EWMAFilter<T>::output() const
{ return value_; }

template <typename T>
void
EWMAFilter<T>::reset()
{ value_ = initial_value_; }

