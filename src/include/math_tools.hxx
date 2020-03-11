#pragma once

template<typename T1, typename T2>
arma::colvec
Math_tools::map_to_arma(std::map<T1, T2> m)
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
T
Math_tools::pythagore_leg(T leg, T hypotenuse)
{
  T leg_2 = std::sqrt(std::pow(hypotenuse, 2) - std::pow(leg, 2));
  return leg_2;
}

template<typename T>
T
Math_tools::pythagore_hypotenuse(T leg_1, T leg_2)
{
  T hypotenuse = std::sqrt(std::pow(leg_1, 2) + std::pow(leg_2, 2));
  return hypotenuse;
}

