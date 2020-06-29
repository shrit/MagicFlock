/**
 * @file main.cc
 * @brief train the dataset
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

#include <ILMR/CLI11.hpp>

/*  locale defined include */
#include "training.hh"

int
main(int argc, char* argv[])
{
  /*  Init logging system */
  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  CLI::App app{ "Example of a neural network trainer using mlpack" };

  std::string dataset_filename = "default";
  bool verbose = false;
  app
    .add_option("-f,--file",
                dataset_filename,
                "Please enter the full path for dataset file name")
    ->required()
    ->check(CLI::ExistingFile);

  app.add_flag("-v, --verbose", verbose, " Make the output more verbose");

  CLI11_PARSE(app, argc, argv);

  if (verbose) {
    spdlog::set_level(spdlog::level::debug);
  }

  logger->info("Start training...");
  Train trainer(std::move(dataset_filename), logger);
  trainer.run();
  logger->info("Finished training...");
}
