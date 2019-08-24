#include <mlpack/core.hpp>
#include <mlpack/core/data/split_data.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <armadillo>

#include "../log.hh"
#include "../settings.hh"

class Train {

public:

  Train();

  double accuracy(const arma::Row<size_t>& predLabels,
		  const arma::Row<size_t>& LabelY);
  
  double accuracy_mse(const arma::mat& predLabels,
		      const arma::mat& LabelY);
  
  arma::Row<size_t> getLabels(const arma::mat& predOut);

  void load_data_set(std::string&& dataset_file);
  void define_label_column_size(int x);
  
  void run(const Settings& settings);
  void classification();
  void regression();
  
  Train(Train const&) = delete;

  Train(Train &&) = default;

private:

  arma::mat dataset_;

  arma::mat trainset_;
  arma::mat testset_;
  
  arma::mat train_features_;
  arma::mat train_labels_;

  arma::mat test_features_;
  arma::mat test_labels_;

  double ratio_ = 0.1;

};
