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
    ILMR::logger::logger_->error(
      "Can not convert to one hot, please add more classes...");
  }
  return one_hot;
}

template<typename Arg>
arma::colvec
OneHotEncoding::to_one_hot_encoding(ignition::math::Vector3d vec,
                                    Arg number_of_class)
{
  arma::colvec one_hot(number_of_class, arma::fill::zeros);
  // The vector need to have round before being treated

  if (vec.X() == 1 and vec.Y() == 0 && vec.Z() == 0) {
    one_hot.at(0) = 1;
  } else if (vec.X() == -1 and vec.Y() == 0 && vec.Z() == 0) {
    one_hot.at(1) = 1;

  } else if (vec.X() == 0 and vec.Y() == 1 && vec.Z() == 0) {
    one_hot.at(2) = 1;

  } else if (vec.X() == 0 and vec.Y() == -1 && vec.Z() == 0) {
    one_hot.at(3) = 1;

  } else if (vec.X() == 0 and vec.Y() == 0 && vec.Z() == 1) {
    one_hot.at(4) = 1;

  } else if (vec.X() == 0 and vec.Y() == 0 && vec.Z() == -1) {
    one_hot.at(5) = 1;

  } else if (vec.X() == 1 and vec.Y() == 1 && vec.Z() == 0) {
    one_hot.at(6) = 1;

  } else if (vec.X() == -1 and vec.Y() == -1 && vec.Z() == 0) {
    one_hot.at(7) = 1;

  } else if (vec.X() == 1 and vec.Y() == -1 && vec.Z() == 0) {
    one_hot.at(8) = 1;

  } else if (vec.X() == -1 and vec.Y() == 1 && vec.Z() == 0) {
    one_hot.at(9) = 1;

  } else if (vec.X() == 0 and vec.Y() == 0 && vec.Z() == 0) {
    one_hot.at(10) = 1;
  }

  return one_hot;
}

template<typename Arg>
arma::uword
OneHotEncoding::from_one_hot_encoding(arma::Col<Arg> col_vec)
{
  return index = col_vec.index_max();
}
