#pragma once

/*  Standard C++ includes  */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <thread>
#include <tuple>
#include <vector>

/*  MLPack includes */
#include <mlpack/core.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>

/* local includes */
#include "ann_predictor.hpp"
#include "argmin.hpp"
#include <IL4MRC/util/logger.hpp>
#include <IL4MRC/controller/quadrotor.hpp>

using namespace ILMR;

template<class QuadrotorType>
class AnnStatePredictor : public virtual AnnPredictor<QuadrotorType>
{
public:
  AnnStatePredictor(QuadrotorType& quad);

  typename QuadrotorType::Action best_predicted_action();
  typename QuadrotorType::Action best_predicted_mig_action();
  typename QuadrotorType::Action best_predicted_cohsep_action();
  
  AnnStatePredictor(AnnStatePredictor const&) = delete;
  AnnStatePredictor(AnnStatePredictor&&) = default;
};

#include "ann_state_predictor_impl.hpp"
