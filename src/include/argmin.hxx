#pragma once

template<class InputType, class OutputType>
Argmin<InputType, OutputType>::Argmin(InputType mat, InputType mat2)
{
  InputType mat3 = mat - mat2;
  mat3 = arma::abs(mat3);
  //logger::logger->info("Final prediction matrix {}", mat3);
  arma::colvec vec = arma::sum(mat3, 1);
  vec = arma::reverse(vec);
  logger::logger_->info("Sum of distances: {}", vec);
  argmin_result_ = vec.index_min();
}

template<class InputType, class OutputType>
Argmin<InputType, OutputType>::Argmin(InputType mat,
                                      InputType mat2,
                                      arma::uword remove_col)
{
  InputType mat3 = mat - mat2;
  mat3 = arma::abs(mat3);
  //logger::logger->info("Final prediction matrix {}", mat3);
  mat3.shed_col(remove_col);
  arma::colvec vec = arma::sum(mat3, 1);
  vec = arma::reverse(vec);
  logger::logger_->info("Sum of distances: {}", vec);
  argmin_result_ = vec.index_min();
}

template<class InputType, class OutputType>
OutputType
Argmin<InputType, OutputType>::result() const
{
  return argmin_result_;
}