#include "training.hh"


Train::Train(){};




Train::load_data_set()
{

  /*  We need to figure a way to load dataset from a file or 
      to get the data set from the generator */

  // Load the training set.

  mlpack::data::Load("new_sample.csv", dataset_, true);   
  
  // Split the labels from the training set.
  trainData_ = dataset.submat(0, 0,
			     dataset.n_rows - 5,
			     dataset.n_cols - 1);
  
  // Split the data from the training set.
  trainLabel_ = dataset.submat(dataset.n_rows - 4, 0,
			       dataset.n_rows - 1, dataset.n_cols - 1);
  
  
  mlpack::data::Load("test.csv", testset_, true);

  testData_ = testset.submat(0, 0,
			    testset.n_rows - 5,
			    testset.n_cols - 1);
  
  testlabel_ = testset.submat(testset.n_rows - 4, 0,
			     testset.n_rows - 1,
			     testset.n_cols - 1);
      
}


Train::run()
{
  FFN<SigmoidCrossEntropyError<>, RandomInitialization> model;
  model.Add<Linear<> >(trainData.n_rows, 200);
  model.Add<LeakyReLU<> >();
  model.Add<Linear<> >(200, 200);
  model.Add<LeakyReLU<> >();
  model.Add<Linear<> >(200, 200);
  model.Add<LeakyReLU<> >();
  model.Add<Linear<> >(200, 4);
  model.Add<LeakyReLU<> >();

  ens::AdamType<ens::AdamUpdate> optimizer;
  
  // Train the model.
  for (int i = 0; i < 150; ++i) {
    std::cout << "training..." << std::endl;
    model.Train(trainData,
		trainLabelsTemp,
		optimizer);
    
    optimizer.ResetPolicy() = false;
    
    arma::mat pred;    
    model.Predict(trainData, pred);
    // Calculate accuracy on training data points.
    Row<size_t> predLabels = getLabels(pred);
    Row<size_t> YLabels = getLabels(trainLabelsTemp);
    
    double trainAccuracy = accuracy(predLabels, YLabels);

    LogInfo() << "Epoch " << i
	      << ":\tTraining Accuracy = "<< trainAccuracy
	      << "%" ;
	      
    
  }

  LogInfo() << "Training Finished...";
  LogInfo() << "Test with Independent part..."
    arma::mat predtest;    

  model.Predict(testData, predtest);  

  Row<size_t> predtestlabel = getLabels(predtest);

  std::cout << predtestlabel.n_cols << std::endl;
  
  Row<size_t> YTestlabels = getLabels(testlabel);

  std::cout << YTestlabels.n_cols << std::endl;

  double testAccuracy = accuracy(predtestlabel, YTestlabels);
  
  std::cout << "Testing Accuracy = "<< testAccuracy<< "%,"
	      << std::endl;
  
  predtest = predtest.t();
  predtest.save("result.csv", raw_ascii);

  mlpack::data::Save("model.xml", "model", model, false);
   
}
