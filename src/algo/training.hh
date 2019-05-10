#include <mlpack/core.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/sigmoid_cross_entropy_error.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <armadillo>


class Train {

public:


  Train();


  double accuracy(arma::Row<size_t> predLabels,
		  arma::Row<size_t> LabelY);
  

  arma::Row<size_t> getLabels(const arma::mat& predOut);
  
  void load_data_set();



  
  void run();
  
  Train(Train const&) = delete;  
  
  Train(Train &&) = default;
  

private:
  
  arma::mat dataset_;
  
  arma::mat trainData_;
  arma::mat trainlabel_;

  arma::mat testset_;
  arma::mat testData_:
  arma::mat testlabel_;


  
};
