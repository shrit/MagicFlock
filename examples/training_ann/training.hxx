#pragma once

template<class simulator_t>
Train<simulator_t>::Train(std::string dataset_filename,
                          std::shared_ptr<spdlog::logger> logger)
  : logger_(logger)
{
  dataset_.load_dataset_file(std::move(dataset_filename));
  dataset_.init_model_directory();
}

template<class simulator_t>
void
Train<simulator_t>::regression()
{
  dataset_.set_label_column_number(3);
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::GlorotInitialization>
    model;
  model.Add<mlpack::ann::Linear<>>(dataset_.train_features().n_rows, 200);
  model.Add<mlpack::ann::LeakyReLU<>>();
  model.Add<mlpack::ann::Linear<>>(200, 200);
  model.Add<mlpack::ann::LeakyReLU<>>();
  model.Add<mlpack::ann::Dropout<>>(0.5);
  model.Add<mlpack::ann::Linear<>>(200, 200);
  model.Add<mlpack::ann::LeakyReLU<>>();
  model.Add<mlpack::ann::Dropout<>>(0.5);
  model.Add<mlpack::ann::Linear<>>(200, 3);

  ens::AdamType<ens::AdamUpdate> optimizer;

  optimizer.ResetPolicy() = false;
  optimizer.Tolerance() = -1;
  optimizer.MaxIterations() = 0;

  Timer timer;
  logger_->info("Starting.....");

  timer.start();
  model.Train(dataset_.train_features(),
              dataset_.train_labels(),
              optimizer,
              ens::PrintLoss(),
              ens::ProgressBar(),
              ens::EarlyStopAtMinLoss(
                dataset_.test_features(), dataset_.test_labels(), 100),
              ens::StoreBestCoordinates<arma::mat>());
  double elapsed_time = timer.stop();
  logger_->info("Training time: {}", elapsed_time, "seconds");

  logger_->info("Training Finished...");
  logger_->info("Test with Independent part...");
  dataset_.save_model(model, "model");
}

template<class simulator_t>
void
Train<simulator_t>::run()
{
  regression();
}
