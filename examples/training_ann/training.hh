#pragma once

#include <mlpack/core.hpp>
#include <mlpack/core/data/split_data.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <ensmallen_bits/callbacks/callbacks.hpp>
#include <armadillo>

#include <ILMR/log.hh>

class Train {

public:

  Train();

  double accuracy(const arma::Row<size_t>& predLabels,
		  const arma::Row<size_t>& LabelY);

  arma::Row<size_t> getLabels(const arma::mat& predOut);
  
  void run(std::string&& name);
  void classification();
  void regression();
  
  Train(Train const&) = delete;

  Train(Train &&) = default;
};
