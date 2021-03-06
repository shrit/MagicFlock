#include "training.hh"

Train::Train(std::string dataset_filename,
             std::shared_ptr<spdlog::logger> logger)
  : logger_(logger)
{
  dataset_.load_dataset_file(std::move(dataset_filename));
  dataset_.init_model_directory();
}

void
Train::regression()
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
  ens::StoreBestCoordinates<arma::mat> bestCoordinates;

  timer.start();
  model.Train(dataset_.train_features(),
              dataset_.train_labels(),
              optimizer,
              ens::PrintLoss(),
              ens::ProgressBar(),
              ens::EarlyStopAtMinLoss(
                [&](const arma::mat& /*param*/) {
                  return model.Evaluate(dataset_.test_features(),
                                        dataset_.test_labels());
                },
                10),
              bestCoordinates);

  double elapsed_time = timer.stop();
  logger_->info("Training time: {}", elapsed_time, "seconds");

  logger_->info("Training Finished...");

  logger_->info(
    "Training loss: {}",
    model.Evaluate(dataset_.train_features(), dataset_.train_labels()));

  model.Parameters() = bestCoordinates.BestCoordinates();
  dataset_.save_model(model, "model");
}

void
Train::run()
{
  regression();
}
