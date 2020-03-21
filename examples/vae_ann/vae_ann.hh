#pragma once

#include <mlpack/core.hpp>
#include <ensmallen_bits/adam/adam_update.hpp>
#include <ensmallen_bits/callbacks/callbacks.hpp>
#include <mlpack/methods/ann/ffn.hpp>
#include <mlpack/methods/ann/init_rules/glorot_init.hpp>
#include <mlpack/methods/ann/layer/layer.hpp>
#include <mlpack/methods/ann/loss_functions/mean_squared_error.hpp>

#include <ILMR/data_set.hh>
#include <ILMR/logger.hh>
#include <ILMR/timer.hh>

template<class simulator_t>
class VAE
{

public:
  VAE(std::string dataset_filename, std::shared_ptr<spdlog::logger> logger);

  void run();
  void regression();

  VAE(VAE const&) = delete;
  VAE(VAE&&) = default;

private:
  DataSet<simulator_t> dataset_;
  std::shared_ptr<spdlog::logger> logger_;
};

#include "vae_ann.hxx"
