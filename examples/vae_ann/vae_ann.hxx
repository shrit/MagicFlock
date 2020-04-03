#pragma once

template<class simulator_t>
VAE<simulator_t>::VAE(std::string dataset_filename,
                      std::shared_ptr<spdlog::logger> logger)
  : logger_(logger)
{
  dataset_.load_dataset_file(std::move(dataset_filename));
  dataset_.init_model_directory();
}

template<class simulator_t>
void
VAE<simulator_t>::regression()
{
  dataset_.set_label_column_number(3);

  mlpack::ann::FFN<mlpack::ann::ReconstructionLoss<
                     arma::mat,
                     arma::mat,
                     mlpack::ann::BernoulliDistribution<arma::mat>>,
                   mlpack::ann::HeInitialization>
    model;

  model.Add<mlpack::ann::IdentityLayer<>>();

  int latentSize = 2;
  // Encoder.
  mlpack::ann::Sequential<>* encoder = new mlpack::ann::Sequential<>();

  encoder->Add<mlpack::ann::Linear<>>(dataset_.train_features().n_rows, 200);
  encoder->Add<mlpack::ann::ReLULayer<>>();
  encoder->Add<mlpack::ann::Linear<>>(200, 200);
  encoder->Add<mlpack::ann::ReLULayer<>>();
  encoder->Add<mlpack::ann::Linear<>>(200, 200);
  encoder->Add<mlpack::ann::ReLULayer<>>();
  encoder->Add<mlpack::ann::Linear<>>(200, 200);
  encoder->Add<mlpack::ann::ReLULayer<>>();
  encoder->Add<mlpack::ann::Linear<>>(200, latentSize);

  model.Add(encoder);

  // Reparametrization layer.
  model.Add<mlpack::ann::Reparametrization<>>(latentSize);

  // Decoder.
  mlpack::ann::Sequential<>* decoder = new mlpack::ann::Sequential<>();

  decoder->Add<mlpack::ann::Linear<>>(latentSize, 200);
  decoder->Add<mlpack::ann::ReLULayer<>>();
  decoder->Add<mlpack::ann::Linear<>>(200, 200);
  decoder->Add<mlpack::ann::ReLULayer<>>();
  decoder->Add<mlpack::ann::Linear<>>(200, 200);
  decoder->Add<mlpack::ann::ReLULayer<>>();
  decoder->Add<mlpack::ann::Linear<>>(200, 200);

  model.Add(decoder);

  ens::AdamType<ens::AdamUpdate> optimizer;

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
VAE<simulator_t>::run()
{
  regression();
}
