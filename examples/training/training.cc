#include "training.hh"


Train::Train(){};

double Train::accuracy(const arma::Row<size_t>& predLabels,
	        const arma::Row<size_t>& LabelY)
{
  // Calculating how many predicted classes are coincide with real labels.
  size_t success = 0;
  for (size_t j = 0; j < LabelY.n_cols; j++) {
    if (predLabels(j) == LabelY(j)) {
      ++success;
    }
  }

  // Calculating percentage of correctly classified data points.
  return (double)success / (double)LabelY.n_cols * 100.0;
}

/*  HACK, Waiting until callback pull request to be merged into ensmallen
    library, meanwhile this calculation is used to print the value of the 
    loss during the training. Print Loss callback in going to be merged
    in the next months*/
double Train::accuracy_mse(const arma::mat& predLabels, const arma::mat& LabelY)
{
  // Calculating how many predicted classes are coincide with real labels.
  return arma::accu(arma::square(predLabels - LabelY)) / LabelY.n_cols;
}

/*  Used only for classification problem to compare values of several classes */
arma::Row<size_t> Train::getLabels(const arma::mat& predOut)
{
  arma::Row<size_t> pred(predOut.n_cols);

  // Class of a j-th data point is chosen to be the one with maximum value
  // in j-th column plus 1 (since column's elements are numbered from 0).
  for (size_t j = 0; j < predOut.n_cols; ++j) {
    pred(j) = arma::as_scalar(arma::find(
					 arma::max(predOut.col(j)) == predOut.col(j), 1)) + 1;
  }
  return pred;
}

void Train::load_data_set(std::string&& dataset_file)
{  
  // Load the training set.
  mlpack::data::Load(dataset_file, dataset_, true);
  mlpack::data::Split(dataset_, trainset_, testset_, ratio_);
}

/* This function define the number of columns used for features and
   labels. Note that, mlpack is column major. Thus, rows are columns   
   @param x is the number of columns used for label*/
void Train::define_label_column_size(int x) {
  // Split the labels from the training set.
  train_features_ = trainset_.submat(0, 0,
				     trainset_.n_rows - (x + 1),
				     trainset_.n_cols - 1);
  
  // Split the data from the training set.
  train_labels_ = trainset_.submat(trainset_.n_rows - x, 0,
				  trainset_.n_rows - x, trainset_.n_cols - 1);
  
  test_features_ = testset_.submat(0, 0,
				   testset_.n_rows - (x + 1),
				   testset_.n_cols - 1);
  
  test_labels_ = testset_.submat(testset_.n_rows - x, 0,
				 testset_.n_rows - x, testset_.n_cols - 1);
}

void Train::classification()
{
  define_label_column_size(4);
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> model;
  model.Add<mlpack::ann::Linear<> >(train_features_.n_rows, 200);
  model.Add<mlpack::ann::LeakyReLU<> >();
  model.Add<mlpack::ann::Linear<> >(200, 200);
  model.Add<mlpack::ann::LeakyReLU<> >();
  model.Add<mlpack::ann::Dropout<> >(0.5);
  model.Add<mlpack::ann::Linear<> >(200, 200);
  model.Add<mlpack::ann::LeakyReLU<> >();
  model.Add<mlpack::ann::Dropout<> >(0.5);
  model.Add<mlpack::ann::Linear<> >(200, 4);
  model.Add<mlpack::ann::LeakyReLU<> >();

  ens::AdamType<ens::AdamUpdate> optimizer;

  // Train the model.
  for (int i = 0; i < 1000; ++i) {
    LogInfo() << "Training..." << "\t Epoch " << i;
    model.Train(train_features_,
		train_labels_,
		optimizer,
		ens::PrintLoss(),
		ens::ProgressBar(),
		ens::StoreBestCoordinates<arma::mat>());

    optimizer.ResetPolicy() = false;

    arma::mat pred;
    model.Predict(train_features_, pred);
    // Calculate accuracy on training data points.
  }
  LogInfo() << "Training Finished...";
  LogInfo() << "Test with Independent part...";

  arma::mat predtest;

  model.Predict(test_features_, predtest);
  model.Evaluate(test_features_, predtest);

  arma::Row<size_t> predtestlabel = getLabels(predtest);

  arma::Row<size_t> YTestlabels = getLabels(test_labels_);

  double testAccuracy = accuracy(predtestlabel, YTestlabels);

  LogInfo() << "Testing Accuracy = "<< testAccuracy<< "%,";
 
  predtest = predtest.t();
  predtest.save("result.csv", arma::raw_ascii);

  mlpack::data::Save("model.xml", "model", model, false);  
}

void Train::regression()
{
  define_label_column_size(4);
  mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
		   mlpack::ann::RandomInitialization> model;
  model.Add<mlpack::ann::Linear<> >(train_features_.n_rows, 200);
  model.Add<mlpack::ann::LeakyReLU<> >();
  model.Add<mlpack::ann::Linear<> >(200, 200);
  model.Add<mlpack::ann::LeakyReLU<> >();
  model.Add<mlpack::ann::Dropout<> >(0.5);
  model.Add<mlpack::ann::Linear<> >(200, 200);
  model.Add<mlpack::ann::LeakyReLU<> >();
  model.Add<mlpack::ann::Dropout<> >(0.5);
  model.Add<mlpack::ann::Linear<> >(200, 4);

  ens::AdamType<ens::AdamUpdate> optimizer;

  // Train the model.
  for (int i = 0; i < 1000; ++i) {
    LogInfo() << "Training..." << "\t Epoch " << i;
    model.Train(train_features_,
		train_labels_,
		optimizer,
		ens::PrintLoss(),
		ens::ProgressBar(),
		ens::StoreBestCoordinates<arma::mat>());
      
    optimizer.ResetPolicy() = false;
      
    arma::mat pred;
    model.Predict(train_features_, pred);
    model.Evaluate(train_features_, pred);          
  }
  LogInfo() << "Training Finished...";
  LogInfo() << "Test with Independent part...";
  
  arma::mat predtest;
    
  model.Predict(test_features_, predtest);
  model.Evaluate(test_features_, predtest);
 
  double testAccuracy = accuracy_mse(predtest, test_labels_);    
  std::cout << "LOSS  = "<< testAccuracy<< "%,"
	    << std::endl;
    
  predtest = predtest.t();
  predtest.save("result.csv", arma::raw_ascii);    
  mlpack::data::Save("model.txt", "model", model, false);  
}

void Train::run(std::string&& name)
{
  if (name == "classification") {
    classification();    
  } else if (name == "regression") {
    regression();
  }      
}
