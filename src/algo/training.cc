#include "training.hh"


Train::Train(){};

double Train::accuracy(arma::Row<size_t> predLabels,
	        arma::Row<size_t> LabelY)
{
  // Calculating how many predicted classes are coincide with real labels.
  size_t success = 0;
  for (size_t j = 0; j < LabelY.n_cols; j++) {
    //    std::cout << predLabels(j) << std::endl;
    //  std::cout << LabelY(j) << std::endl;
    if (predLabels(j) == LabelY(j)) {
      ++success;
    }
  }
  
  // Calculating percentage of correctly classified data points.
  return (double)success / (double)LabelY.n_cols * 100.0;      
}

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

void Train::load_data_set()
{

  /*  We need to figure a way to load dataset from a file or 
      to get the data set from the generator */

  // Load the training set.

  mlpack::data::Load("new_sample.csv", dataset_, true);   
  
  // Split the labels from the training set.
  trainData_ = dataset_.submat(0, 0,
			     dataset_.n_rows - 5,
			     dataset_.n_cols - 1);
  
  // Split the data from the training set.
  trainlabel_ = dataset_.submat(dataset_.n_rows - 4, 0,
			       dataset_.n_rows - 1, dataset_.n_cols - 1);
  
  
  mlpack::data::Load("test.csv", testset_, true);

  testData_ = testset_.submat(0, 0,
			    testset_.n_rows - 5,
			    testset_.n_cols - 1);
  
  testlabel_ = testset_.submat(testset_.n_rows - 4, 0,
			     testset_.n_rows - 1,
			     testset_.n_cols - 1);
      
}

void Train::run()
{
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> model;
  model.Add<mlpack::ann::Linear<> >(trainData_.n_rows, 200);
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
  for (int i = 0; i < 54; ++i) {
    std::cout << "training..." << std::endl;
    model.Train(trainData_,
		trainlabel_,
		optimizer);
    
    optimizer.ResetPolicy() = false;
    
    arma::mat pred;    
    model.Predict(trainData_, pred);
    // Calculate accuracy on training data points.
    arma::Row<size_t> predLabels = getLabels(pred);
    arma::Row<size_t> YLabels = getLabels(trainlabel_);
    
    double trainAccuracy = accuracy(predLabels, YLabels);

    LogInfo() << "Epoch " << i
	      << ":\tTraining Accuracy = "<< trainAccuracy
	      << "%" ;          
  }

  LogInfo() << "Training Finished...";
  LogInfo() << "Test with Independent part...";
  
  arma::mat predtest;   
  
  model.Predict(testData_, predtest);  
  
  arma::Row<size_t> predtestlabel = getLabels(predtest);
  
  std::cout << predtestlabel.n_cols << std::endl;
  
  arma::Row<size_t> YTestlabels = getLabels(testlabel_);

  std::cout << YTestlabels.n_cols << std::endl;

  double testAccuracy = accuracy(predtestlabel, YTestlabels);
  
  std::cout << "Testing Accuracy = "<< testAccuracy<< "%,"
	      << std::endl;
  
  predtest = predtest.t();
  predtest.save("result.csv", arma::raw_ascii);

  mlpack::data::Save("model.xml", "model", model, false);   
}
