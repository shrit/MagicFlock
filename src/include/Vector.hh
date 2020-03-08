#pragma once


#include <vector>

class VectorHelper
{
  VectorHelper(){}

  template<typename Arg>
  Arg variance(std::vector<Arg> vec);

  template<typename Arg>
  Arg mean(std::vector<Arg> vec);

  template<typename Arg>
  std::vector<int> fill_range(Arg value);

  template<typename T>
  long long unsigned int index_of_max_value(const std::vector<T>& vec);

  template<typename T>
  long long unsigned int index_of_min_value(const std::vector<T>& vec);  
};

