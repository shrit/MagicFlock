#pragma once

#include <mlpack/core.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <ensmallen_bits/callbacks/callbacks.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/init_rules/glorot_init.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>

#include <IL4MRC/data/dataset.hpp>
#include <IL4MRC/util/logger.hpp>
#include <IL4MRC/util/time.hpp>

class Train
{

public:
  Train(std::string dataset_filename, std::shared_ptr<spdlog::logger> logger);

  void run();
  void regression();

  Train(Train const&) = delete;
  Train(Train&&) = default;

private:
  DataSet dataset_;
  std::shared_ptr<spdlog::logger> logger_;
};

