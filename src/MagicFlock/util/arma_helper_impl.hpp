#pragma once

template<typename T1, typename T2>
arma::colvec
ArmaHelper::map_to_arma(std::map<T1, T2> m)
{
  std::vector<T2> vec;
  for (typename std::map<T1, T2>::iterator it = m.begin(); it != m.end();
       ++it) {
    vec.push_back(it->second);
  }
  arma::colvec arma_vec = arma::conv_to<arma::colvec>::from(vec);

  return arma_vec;
}

template<typename T>
arma::colvec
ArmaHelper::vec_to_arma(std::vector<T> vec)
{
  return arma::conv_to<arma::colvec>::from(vec);
}
