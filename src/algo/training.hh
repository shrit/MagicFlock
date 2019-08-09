#include <mlpack/core.hpp>
#include <mlpack/core/data/split_data.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <armadillo>

#include "../log.hh"

class Train {

public:

  Train();

  double accuracy(const arma::Row<size_t>& predLabels,
		  const arma::Row<size_t>& LabelY);

  arma::Row<size_t> getLabels(const arma::mat& predOut);

  void load_data_set(std::string&& dataset_file);

  void run();

  Train(Train const&) = delete;

  Train(Train &&) = default;

private:

  arma::mat dataset_;

  arma::mat trainset_;
  arma::mat testset_;
  
  arma::mat trainData_;
  arma::mat trainlabel_;

  arma::mat testData_;
  arma::mat testlabel_;

  double ratio_ = 0.1;

};
