# pragma once

#include <map>
#include <vector>

class Histogram
{
public:

  template<typename T>
  std::map<T, T> get_histogram();

  template<typename T>
  void histogram(T times);

  template<typename KeyType, typename ValueType>
  std::pair<KeyType, ValueType> get_max_histogram(
    const std::map<KeyType, ValueType>& x);

private:
  std::map<int, int> histo_; 
  
};

#include "histogram_impl.hpp"
