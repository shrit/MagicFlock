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
#include "action.hh"
#include "global.hh"
#include "plot.hh"

namespace lt = local_types;

template<class simulator_t>
class DataSet
{

public:
  DataSet();

  void parse_dataset_file(std::string file_name);
  void load_dataset_file(std::string&& file_name);
  void load_knn_dataset(std::string file_name);
  void set_label_column_number(int x);

  /* All of the following functions need to be refactored*/
  template<typename Arg, typename... Args>
  void save_evaluation(Arg&& arg, Args&&... args);

  template<typename Arg, typename... Args>
  void save_csv_data_set(Arg&& arg, Args&&... args);

  template<typename Arg, typename... Args>
  void save_csv_data_set_2_file(std::string file_name,
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

  template<typename ModelType>
  void save_model(ModelType&& model, std::string model_name);

  void init_model_directory();

  template<typename Arg, typename... Args>
  void plot(std::string title,
            std::string xlabel,
            std::string ylabel,
            Arg arg,
            Arg argv,
            Args... args);

  void init_dataset_directory();

  arma::mat submat_using_indices(arma::mat, arma::Mat<size_t>);
  arma::mat st_mat() const;
  arma::mat at_mat() const;
  arma::mat st_1_mat() const;
  arma::mat at_1_mat() const;
  arma::mat st_2_mat() const;
  arma::mat s_a_s_t_1_mat() const;

  arma::mat dataset() const;
  arma::mat test_features() const;
  arma::mat test_labels() const;
  arma::mat train_features() const;
  arma::mat train_labels() const;
  arma::mat trainset() const;
  arma::mat testset() const;

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

  std::string model_file_name_;

  arma::mat st_mat_;
  arma::mat at_mat_;
  arma::mat st_1_mat_;
  arma::mat at_1_mat_;
  arma::mat st_2_mat_;
  arma::mat s_a_s_t_1_mat_;
};

#include "data_set.hxx"
