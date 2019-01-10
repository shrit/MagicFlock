#include <cassert>


template <typename type>
class EMA {
    
public:

  EMA(const type&	alpha,
      const type&	initial_value);

  /// Apply the smoothing on a new value, store and return the result.
  type	        filtered_values(const value_type& new_value);
  
  
private:
  const type            alpha_;		//< Smoothing coefficient.
  const type	initial_value_;	//< Initial value.
  type		value_;		//< Stored value.

};



