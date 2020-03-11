#pragma once

/*  Simple implementation, need more logical one */
template<typename Arg, typename Arg2>
arma::colvec
Math_tools::to_one_hot_encoding(Arg arg, Arg2 number_of_class)
{
  arma::colvec one_hot(number_of_class, arma::fill::zeros);

  if (number_of_class > static_cast<int>(arg)) {
    one_hot.at(static_cast<int>(arg)) = 1;
  } else {
    logger::logger_->error(
      "Can not convert to one hot, please add more classes...");
  }
  return one_hot;
}

template<typename Arg>
arma::uword
Math_tools::from_one_hot_encoding(arma::Col<Arg> col_vec)
{
  return index = col_vec.index_max();
}

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

