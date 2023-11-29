#pragma once

#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

/*  Boost library include */
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/* MLPack includes*/
#include <mlpack/core.hpp>
#include <mlpack/core/data/split_data.hpp>

/*  local defined include */
#include <MagicFlock/actions/discret_actions.hpp>
#include <MagicFlock/util/Vector.hpp>

class DataSet
{

public:
  DataSet();

  void parse_dataset_file(std::string file_name)
  {
    dataset_.load(file_name, arma::csv_ascii);

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

  void load_dataset_file(std::string&& file_name)
  {
    // Load the training set.
    mlpack::data::Load(dataset_file, dataset_, true);
    mlpack::data::Split(dataset_, trainset_, testset_, ratio_);
  }
  
  void load_knn_dataset(std::string file_name)
  {
    // Load the training set.
    mlpack::data::Load(dataset_file, dataset_knn_, true);
    s_a_s_t_1_mat_ = dataset_knn_.submat(0, 0, 14, dataset_knn_.n_cols - 1);
    logger::logger_->info(
      "First line of knn_dataset:  {}",
      s_a_s_t_1_mat_(arma::span(0, s_a_s_t_1_mat_.n_rows - 1), arma::span(0, 0)));
  }

  /**
   * This function define the number of columns used for features and
   * labels. Note that, mlpack is column major. Thus, rows are columns
   *
   * @param x is the number of columns used for label
   */
  void set_label_column_number(int x)
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

  /* All of the following functions need to be refactored*/
  template<typename Arg, typename... Args>
  void save_evaluation(Arg&& arg, Args&&... args);

  template<typename Arg, typename... Args>
  void save_csv_dataset(Arg&& arg, Args&&... args);

  template<typename Arg, typename... Args>
  void save_csv_dataset_2_file(std::string file_name,
                               Arg&& arg,
                               Args&&... args);

  template<typename Arg, typename... Args>
  void save_error_file(std::string file_name, Arg&& arg, Args&&... args);

  template<typename Arg>
  void save_histogram(Arg&& arg);

  template<typename Arg>
  void save_controller_count(Arg&& arg);

  template<typename Arg>
  void save_episodes(Arg&& arg);

  template<typename Arg>
  void save_actions(std::string file_name, Arg&& arg);

  template<typename ModelType>
  void save_model(ModelType&& model, std::string model_name);

  template<typename Arg>
  void save_state(std::string file_name, Arg&& arg);

  void init_model_directory()
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
                       "/model" + time_stream.str() + ".bin";
  }
 
  void init_dataset_directory()
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

  arma::mat submat_using_indices(arma::mat& matrix_to_sub, arma::Mat<size_t>)
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
 
  arma::mat st_mat() const
  {
    return st_mat_;
  }

  arma::mat at_mat() const
  {
    return at_mat_;
  }

  arma::mat st_1_mat() const
  {
    return st_1_mat_;
  }

  arma::mat at_1_mat() const
  {
    return at_1_mat_;
  }

  arma::mat st_2_mat() const
  {
    return st_2_mat_;
  }

  arma::mat s_a_s_t_1_mat() const
  {
    return s_a_s_t_1_mat_;
  }

  arma::mat dataset() const;
  {
    return dataset_;
  }

  arma::mat test_features() const
  {
    return testset_;
  }

  arma::mat test_labels() const
  {
    return test_labels_;
  }

  arma::mat train_features() const
  {
    return train_features_;
  }

  arma::mat train_labels() const
  {
    return train_labels_;
  }

  arma::mat trainset() const
  {
    return trainset_;
  }

  arma::mat testset() const
  {
    return testset_;
  }

private:
  arma::mat dataset_;
  arma::mat dataset_knn_;
  arma::mat trainset_;
  arma::mat testset_;

  arma::mat train_features_;
  arma::mat train_labels_;

  arma::mat test_features_;
  arma::mat test_labels_;

  double ratio_ = 0.2;

  std::string dataset_file_name_;
  std::string error_file_name_;
  std::string result_file_name_;
  std::string count_file_name_;
  std::string histogram_file_name_;
  std::string readme_file_name_;
  std::string evaluate_file_name_;
  std::string episodes_file_name_;
  std::string action_file_name_;
  std::string model_file_name_;
  std::string state_file_name_;

  arma::mat st_mat_;
  arma::mat at_mat_;
  arma::mat st_1_mat_;
  arma::mat at_1_mat_;
  arma::mat st_2_mat_;
  arma::mat s_a_s_t_1_mat_;

  VectorHelper vec_;
};

#include "dataset_impl.hpp"
