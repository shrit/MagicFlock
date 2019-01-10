
# include "ema_filter.hh"


template <typename type>
EMA<type>::EMA(const type&	alpha,
	       const type&	initial_value)
  : alpha_(alpha),
    initial_value_(initial_value),
    value_(initial_value)
{ assert(alpha > 0 and alpha <= 1); }

template <typename type>
typename EMA<type>::type
EMA<type>::filtered_value(const type& new_value)
{ return value_ = alpha_ * new_value + (1 - alpha_) * value_; }
