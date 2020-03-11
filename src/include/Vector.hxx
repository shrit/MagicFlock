#pragma once

template<typename T>
long long unsigned int
VectorHelper::index_of_max_value(const std::vector<T>& vec)
{
  auto highest = std::max_element(vec.begin(), vec.end());
  return std::distance(vec.begin(), highest);
}

template<typename T>
long long unsigned int
VectorHelper::index_of_min_value(const std::vector<T>& vec)
{
  auto highest = std::min_element(vec.begin(), vec.end());
  return std::distance(vec.begin(), highest);
}

template<typename Arg>
std::vector<int>
VectorHelper::fill_range(Arg value)
{
  std::vector<int> fill;
  for (int i = 0; i < value; ++i) {
    fill.push_back(i);
  }
  return fill;
}

template<typename Arg>
Arg
VectorHelper::mean(std::vector<Arg> vec)
{
  size_t sz = vec.size();
  if (sz == 0)
    return -1;

  return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}

template<typename Arg>
std::vector<double>
VectorHelper::to_std_vector(Arg arg)
{
  std::vector<double> vec;
  if (arg.is_empty()) {
    logger::logger_->error("Can not convert empty vector to std vector");
  }
  if (std::is_same<Arg, arma::rowvec>::value) {
    vec.resize(arg.n_cols);
    vec = arma::conv_to<std::vector<double>>::from(arg);
  } else if (std::is_same<Arg, arma::colvec>::value) {
    vec.resize(arg.n_rows);
    vec = arma::conv_to<std::vector<double>>::from(arg);
  }
  logger::logger_->debug("Converted to std vector: {}", vec);
  return vec;
}

template<typename T1, typename T2>
std::vector<T2>
VectorHelper::map_to_vector(std::map<T1, T2> m)
{
  std::vector<T2> vec;
  for (typename std::map<T1, T2>::iterator it = m.begin(); it != m.end();
       ++it) {
    vec.push_back(it->second);
  }
  return vec;
}

template<typename Arg>
Arg
VectorHelper::variance(std::vector<Arg> vec)
{
  /*  note that diff_f3_ was used here */
  size_t sz = vec.size();
  if (sz == 1)
    return -1;

  /*  Do not take the first value */
  Arg mean = std::accumulate(vec.begin() + 1, vec.end(), 0.0) / vec.size();

  return std::accumulate(vec.begin(),
                         vec.end(),
                         0.0,
                         [&mean, &sz](double accumulator, const double& val) {
                           return accumulator +
                                  ((val - mean) * (val - mean) / (sz - 1));
                         });
}
