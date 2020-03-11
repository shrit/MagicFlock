#pragma once

/*  Simple implementation, need more logical one */
template<typename Arg, typename Arg2>
arma::colvec
OneHotEncoding::to_one_hot_encoding(Arg arg, Arg2 number_of_class)
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
OneHotEncoding::from_one_hot_encoding(arma::Col<Arg> col_vec)
{
  return index = col_vec.index_max();
}

