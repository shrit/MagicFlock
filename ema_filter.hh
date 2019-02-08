#ifndef EMA_FILTER_HH_
#define EMA_FILTER_HH_

#include <cassert>

/// Compute the EWMA of the input values.
template <typename T>
class EWMAFilter {
public:
  using value_type = T;

  /**
   * @brief Contructor
   *
   * @param[in] alpha		The smoothing coefficient between 0 and 1.
   *				The lower the coefficient, the smoother
   *				the result.
   * @param[in] initial_value	Initial value to use.
   */
  EWMAFilter(const value_type&	alpha,
	     const value_type&	initial_value = value_type());

  /// Apply the smoothing on a new value, store and return the result.
  value_type		input(const value_type& new_value);

  /// Retrieve the stored value.
  const value_type&	output() const;

  /// Reset the stored value to the initial value.
  void		reset();
private:
  const value_type	alpha_;		//< Smoothing coefficient.
  const value_type	initial_value_;	//< Initial value.
  value_type		value_;		//< Stored value.
};


# include "ema_filter.hxx"

#endif
