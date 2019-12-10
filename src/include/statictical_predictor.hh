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
  
  void predict();
  Actions::Action get_predicted_action();
  
  StatisticalPredictor(StatisticalPredictor const&) = delete;
  StatisticalPredictor(StatisticalPredictor &&) = default;
  
private:
  Math_tools mtools_;
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;
  
};

# include "statictical_predictor.hxx"
