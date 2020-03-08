#pragma once

template<class simulator_t>
DataSet<simulator_t>::DataSet()
{}

template<class simulator_t>
void
DataSet<simulator_t>::parse_dataset_file(std::string file_name)
{
  dataset_.load(file_name, arma::csv_ascii);

  // We need to create a state from the first 3 column from a matrix
  // It is much better to parse data set line by line into state, actions,
  // nextstate , etc
  /*  Then all the parsed state and actions can be agglomerated into matries.
   Then it should be easy to read each line of these matrices since they are
   seperated */

  // /*  Get the indices for state, actions etc... */
  /*  This should be the standard method to parse the dataset */
  // for (int i = 0; i < dataset_.n_cols; ++i) {
  //   if ((dataset_(0,i) ==  and (dataset_(0,i) == 0  or dataset_(0,i) == 1)) {

  //   }
  // }

  arma::mat st_mat =
    dataset_(arma::span(0, dataset_.n_rows - 1), arma::span(0, 3));
  arma::mat at_mat =
    dataset_(arma::span(0, dataset_.n_rows - 1), arma::span(4, 10));
  arma::mat st_1_mat =
    dataset_(arma::span(0, dataset_.n_rows - 1), arma::span(11, 14));
  arma::mat at_1_mat =
    dataset_(arma::span(0, dataset_.n_rows - 1), arma::span(15, 21));
  arma::mat st_2_mat =
    dataset_(arma::span(0, dataset_.n_rows - 1), arma::span(22, 25));

  logger::logger_->debug("Matrix: S_t => t=0: {}", st_mat);

  st_mat_ = st_mat.t();
  at_mat_ = at_mat.t();
  st_1_mat_ = st_1_mat.t();
  at_1_mat_ = at_1_mat.t();
  st_2_mat_ = st_2_mat.t();
}

template<class simulator_t>
void
DataSet<simulator_t>::load_knn_dataset(std::string dataset_file)
{
  // Load the training set.
  mlpack::data::Load(dataset_file, dataset_knn_, true);
  s_a_s_t_1_mat_ = dataset_knn_.submat(0, 0, 14, dataset_knn_.n_cols - 1);
  logger::logger_->info(
    "First line of knn_dataset:  {}",
    s_a_s_t_1_mat_(arma::span(0, s_a_s_t_1_mat_.n_rows - 1), arma::span(0, 0)));
}

template<class simulator_t>
void
DataSet<simulator_t>::load_dataset_file(std::string&& dataset_file)
{
  // Load the training set.
  mlpack::data::Load(dataset_file, dataset_, true);
  mlpack::data::Split(dataset_, trainset_, testset_, ratio_);
}

/* This function define the number of columns used for features and
   labels. Note that, mlpack is column major. Thus, rows are columns
   @param x is the number of columns used for label*/
template<class simulator_t>
void
DataSet<simulator_t>::set_label_column_number(int x)
{
  // Split the labels from the training set.
  train_features_ =
    trainset_.submat(0, 0, trainset_.n_rows - (x + 1), trainset_.n_cols - 1);

  // Split the data from the training set.
  train_labels_ = trainset_.submat(
    trainset_.n_rows - x, 0, trainset_.n_rows - 1, trainset_.n_cols - 1);

  test_features_ =
    testset_.submat(0, 0, testset_.n_rows - (x + 1), testset_.n_cols - 1);

  test_labels_ = testset_.submat(
    testset_.n_rows - x, 0, testset_.n_rows - 1, testset_.n_cols - 1);
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::dataset() const
{
  return dataset_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::trainset() const
{
  return trainset_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::testset() const
{
  return testset_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::train_features() const
{
  return train_features_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::train_labels() const
{
  return train_labels_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::test_labels() const
{
  return test_labels_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::test_features() const
{
  return test_features_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::st_mat() const
{
  return st_mat_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::at_mat() const
{
  return at_mat_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::st_1_mat() const
{
  return st_1_mat_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::at_1_mat() const
{
  return at_1_mat_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::st_2_mat() const
{
  return st_2_mat_;
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::s_a_s_t_1_mat() const
{
  return s_a_s_t_1_mat_;
}

template<class simulator_t>
void
DataSet<simulator_t>::init_dataset_directory()
{
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

  std::stringstream date_stream, time_stream;

  date_stream << now.date();
  time_stream << now.time_of_day();

  std::experimental::filesystem::create_directory("../dataset");
  std::experimental::filesystem::create_directory("../dataset/" +
                                                  date_stream.str());
  std::experimental::filesystem::create_directory(
    "../dataset/" + date_stream.str() + "/" + time_stream.str());

  dataset_file_name_ = "../dataset/" + date_stream.str() + "/" +
                       time_stream.str() + "/" + time_stream.str();

  error_file_name_ = "../dataset/" + date_stream.str() + "/" +
                     time_stream.str() + "/error" + time_stream.str();

  histogram_file_name_ = "../dataset/" + date_stream.str() + "/" +
                         time_stream.str() + "/histogram" + time_stream.str();

  count_file_name_ = "../dataset/" + date_stream.str() + "/" +
                     time_stream.str() + "/count" + time_stream.str();

  result_file_name_ = "../dataset/" + date_stream.str() + "/" +
                      time_stream.str() + "/figure" + time_stream.str();

  readme_file_name_ = "../dataset/" + date_stream.str() + "/" +
                      time_stream.str() + "/readme" + time_stream.str();

  evaluate_file_name_ = "../dataset/" + date_stream.str() + "/" +
                        time_stream.str() + "/evaluate" + time_stream.str();

  episodes_file_name_ = "../dataset/" + date_stream.str() + "/" +
                        time_stream.str() + "/episodes" + time_stream.str();

  action_file_name_ = "../dataset/" + date_stream.str() + "/" +
                      time_stream.str() + "/action" + time_stream.str();
}

template<class simulator_t>
void
DataSet<simulator_t>::init_model_directory()
{
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

  std::stringstream date_stream, time_stream;

  date_stream << now.date();
  time_stream << now.time_of_day();

  std::experimental::filesystem::create_directory("../model");
  std::experimental::filesystem::create_directory("../model/" +
                                                  date_stream.str());
  std::experimental::filesystem::create_directory(
    "../model/" + date_stream.str() + "/" + time_stream.str());

  model_file_name_ = "../model/" + date_stream.str() + "/" + time_stream.str() +
                     "/model" + time_stream.str();
}

template<class simulator_t>
template<typename ModelType>
void
DataSet<simulator_t>::save_model(ModelType&& model, std::string model_name)
{
  mlpack::data::Save(
    model_file_name_, model_name, std::forward<ModelType>(model), false);
}

template<class simulator_t>
template<typename Arg>
void
DataSet<simulator_t>::save_state(std::string file_name, Arg&& arg)
{
  std::ofstream file;
  file.open(state_file_name, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);

  file << "\n";
  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg, typename... Args>
void
DataSet<simulator_t>::save_evaluation(Arg&& arg, Args&&... args)
{
  std::ofstream file;
  file.open(evaluate_file_name_, std::ios::out);

  file << std::forward<Arg>(arg);
  ((file << std::forward<Args>(args)), ...);
  file << "\n";
  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg, typename... Args>
void
DataSet<simulator_t>::save_csv_data_set(Arg&& arg, Args&&... args)
{
  std::ofstream file;
  file.open(dataset_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file << "," << std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg, typename... Args>
void
DataSet<simulator_t>::save_csv_data_set_2_file(std::string file_name,
                                               Arg&& arg,
                                               Args&&... args)
{
  std::ofstream file;
  file.open(dataset_file_name_ + "_" + file_name + ".csv",
            std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file << "," << std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg, typename... Args>
void
DataSet<simulator_t>::save_error_file(std::string file_name,
                                      Arg&& arg,
                                      Args&&... args)
{
  std::ofstream file;
  file.open(error_file_name_ + "_" + file_name + ".csv",
            std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file << "," << std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg>
void
DataSet<simulator_t>::save_histogram(Arg&& arg)
{
  std::ofstream file;
  file.open(histogram_file_name_, std::ios::out);

  for (auto& [key, value] : arg) {
    file << key << " " << value << "\n";
  }
  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg>
void
DataSet<simulator_t>::save_controller_count(Arg&& arg)
{
  std::ofstream file;
  file.open(count_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) << "\n";

  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg>
void
DataSet<simulator_t>::save_episodes(Arg&& arg)
{
  std::ofstream file;
  file.open(episodes_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) << "\n";

  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg>
void
DataSet<simulator_t>::save_actions(std::string file_name, Arg&& arg)
{
  std::ofstream file;
  file.open(action_file_name_ + "_" + file_name, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) << "\n";

  file.flush();
  file.close();
}

template<class simulator_t>
template<typename Arg>
void
DataSet<simulator_t>::plot(std::string title,
                           std::string xlabel,
                           std::string ylabel,
                           std::string file_name,
                           std::string type,
                           Arg x,
                           Arg y)
{
  /*  Gnuplot Config */
  plotcpp::Plot plt(false);
  plt.SetTerminal("svg");
  plt.SetOutput(file_name);
  plt.SetTitle(title);
  plt.SetXLabel(xlabel);
  plt.SetYLabel(ylabel);
  plt.SetAutoscale();

  /*  Need to look into it... */
  if (type == "lines")
    plt.Draw2D(Lines(arg.begin(), arg.end(), argv.begin(), "lines"));
  else if (type == "points")
    plt.Draw2D(Points(arg.begin(), arg.end(), argv.begin(), "lines"));

  plt.Flush();
}

template<class simulator_t>
arma::mat
DataSet<simulator_t>::submat_using_indices(arma::mat matrix_to_sub,
                                           arma::Mat<size_t> indices)
{
  arma::rowvec row;
  arma::mat submatrix;
  for (arma::uword i = 0; i < indices.n_rows; ++i) {
    row = matrix_to_sub(arma::span(indices(i, 0), indices(i, 0)),
                        arma::span(0, matrix_to_sub.n_cols - 1));
    submatrix.insert_rows(0, row);
  }
  return submatrix;
}
