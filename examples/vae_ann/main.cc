/**
 * @file main.cc
 * @brief Apply a variational auto encoder on a dataset
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <CLI/CLI.hpp>

/*  locale defined include */
#include "vae_ann.hh"

#include <ILMR/gazebo.hh>
#include <ILMR/logger.hh>

int
main(int argc, char* argv[])
{
  /*  Init logging system */
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  CLI::App app{ "Example of a varational autoencoder using MLPack" };

  std::string dataset_filename = "default";
  app
    .add_option("-f,--file",
                dataset_filename,
                "Please enter the full path for dataset file name")
    ->required()
    ->check(CLI::ExistingFile);

  CLI11_PARSE(app, argc, argv);

  logger->info("Start training...");
  Train<Gazebo> trainer(std::move(dataset_filename), logger);
  trainer.run();
  logger->info("Finished training...");
}
