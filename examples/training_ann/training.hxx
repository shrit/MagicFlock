template<class simulator_t>
Train<simulator_t>::Train(std::string dataset_filename,
                          std::shared_ptr<spdlog::logger> logger)
  : logger_(logger)
{
  dataset_.load_dataset_file(std::move(dataset_filename));
}

template<class simulator_t>
void
Train<simulator_t>::classification()
{
  dataset_.set_label_column_number(4);
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
                   mlpack::ann::RandomInitialization>
    model;
  model.Add<mlpack::ann::Linear<>>(dataset_.train_features().n_rows, 200);
  model.Add<mlpack::ann::LeakyReLU<>>();
  model.Add<mlpack::ann::Linear<>>(200, 200);
  model.Add<mlpack::ann::LeakyReLU<>>();
  model.Add<mlpack::ann::Dropout<>>(0.5);
  model.Add<mlpack::ann::Linear<>>(200, 200);
  model.Add<mlpack::ann::LeakyReLU<>>();
  model.Add<mlpack::ann::Dropout<>>(0.5);
  model.Add<mlpack::ann::Linear<>>(200, 4);
  model.Add<mlpack::ann::LeakyReLU<>>();

  ens::AdamType<ens::AdamUpdate> optimizer;

  optimizer.Tolerance() = -1;
  optimizer.MaxIterations() = 0;
  model.Train(dataset_.train_features(),
              dataset_.train_labels(),
              optimizer,
              ens::PrintLoss(),
              ens::ProgressBar(),
              ens::StoreBestCoordinates<arma::mat>());

  optimizer.ResetPolicy() = false;

  arma::mat pred;
  model.Predict(dataset_.train_features(), pred);
  // Calculate accuracy on training data points.

  logger_->info("Training Finished...");
  logger_->info("Test with Independent part...");

  arma::mat predtest;

  model.Predict(dataset_.test_features(), predtest);
  double test_accuracy = model.Evaluate(dataset_.test_features(), predtest);

  logger_->info("Testing Accuracy = ", test_accuracy, "%,");

  predtest = predtest.t();
  predtest.save("result.csv", arma::raw_ascii);

  mlpack::data::Save("model.xml", "model", model, false);
}

template<class simulator_t>
void
Train<simulator_t>::regression()
{
  dataset_.set_label_column_number(3);
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
                   mlpack::ann::RandomInitialization>
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

  optimizer.Tolerance() = -1;
  optimizer.MaxIterations() = 0;
  logger_->info("Starting.....");
  Timer timer;
  timer.start();
  model.Train(dataset_.train_features(),
              dataset_.train_labels(),
              optimizer,
              ens::PrintLoss(),
              ens::ProgressBar(),
              ens::EarlyStopAtMinLoss(75),
              ens::StoreBestCoordinates<arma::mat>());
  double elapsed_time = timer.stop();

  logger_->info("Training time: {}", elapsed_time, "seconds");
  optimizer.ResetPolicy() = false;

  logger_->info("Training Finished...");
  logger_->info("Test with Independent part...");

  arma::mat predtest;

  model.Predict(dataset_.test_features(), predtest);
  double testAccuracy = model.Evaluate(dataset_.test_features(), predtest);

  logger_->info("Loss on test set = ", testAccuracy, "%,");

  predtest = predtest.t();
  predtest.save("result.csv", arma::raw_ascii);
  mlpack::data::Save("model.txt", "model", model, false);
}

template<class simulator_t>
void
Train<simulator_t>::run(std::string&& name)
{
  if (name == "classification") {
    classification();
  } else if (name == "regression") {
    regression();
  }
}
