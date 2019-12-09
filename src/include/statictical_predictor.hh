#pragma once

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <numeric>
# include <tuple>
# include <vector>

/*  Armadillo includes  */
# include <armadillo>


/* local includes  */
# include "global.hh"
# include "log.hh"
# include "math_tools.hh"
# include "quadrotor.hh"


namespace lt = local_types;


class StatisticalPredictor
{
public:
  StatisticalPredictor(std::string dataset_file);
  
  std::tuple<arma::mat, arma::uword, Actions::Action> predict(arma::mat& features);
  Actions::Action get_predicted_action();
  
  StatisticalPredictor(StatisticalPredictor const&) = delete;
  StatisticalPredictor(StatisticalPredictor &&) = default;
  
private:
  
  arma::mat dataset;
  
};

# include "statictical_predictor.hxx"
