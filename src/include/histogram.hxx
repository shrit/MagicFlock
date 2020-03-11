#pragma once

template<typename T>
void
Histogram::histogram(T times)
{
  auto it = histo_.find(times);
  if (it != histo_.end()) {
    it->second = it->second + 1;
  } else {
    histo_.insert(std::pair<T, T>(times, 1));
  }
}

template<typename T>
std::map<T, T>
Histogram::get_histogram()
{
  return histo_;
}

template<typename KeyType, typename ValueType>
std::pair<KeyType, ValueType>
Histogram::get_max_histogram(const std::map<KeyType, ValueType>& x)
{
  return *std::max_element(x.begin(),
                           x.end(),
                           [](const std::pair<KeyType, ValueType>& p1,
                              const std::pair<KeyType, ValueType>& p2) {
                             return p1.second < p2.second;
                           });
}
