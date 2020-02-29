#pragma once

#include <mlpack/core.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <ensmallen_bits/callbacks/callbacks.hpp>
#include <mlpack/core/data/split_data.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>
#include <mlpack/methods/ann/init_rules/glorot_init.hpp>

#include <ILMR/data_set.hh>
#include <ILMR/logger.hh>
#include <ILMR/timer.hh>

template<class simulator_t>
class Train
{

public:
  Train(std::string dataset_filename,
        std::shared_ptr<spdlog::logger> logger);

  void run();
  void regression();

  Train(Train const&) = delete;
  Train(Train&&) = default;

private:
 DataSet<simulator_t> dataset_;
 std::shared_ptr<spdlog::logger> logger_;
};

# include "training.hxx"
