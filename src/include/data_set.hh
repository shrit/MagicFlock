#pragma once

# include <algorithm>
# include <chrono>
# include <ctime>
# include <fstream>
# include <experimental/filesystem>
# include <iomanip>
# include <iostream>
# include <iterator>
# include <sstream>
# include <string>
# include <vector>
# include <unordered_map>
# include <utility>

/*  Boost library include */
# include <boost/algorithm/string.hpp>
# include <boost/date_time/posix_time/posix_time.hpp>

/* MLPack includes*/
#include <mlpack/core/data/split_data.hpp>

/*  local defined include */
# include "action.hh"
# include "state.hh"
# include "global.hh"
# include "plot.hh"

# include <armadillo>

namespace lt = local_types;

template <class simulator_t>
class DataSet {

public:

  DataSet();

  void parse_dataset_file(std::string file_name);
  void load_dataset_file(std::string&& file_name);
  void set_label_column_number(int x);
  
  template <typename Arg, typename... Args>
  void write_data_set_file(std::ofstream& file, Arg&& arg, Args&&... args);

  template <typename Arg, typename... Args>
  void save_csv_data_set(Arg&& arg, Args&&... args);

  template <typename Arg, typename... Args>
  void save_csv_data_set_2_file(std::string file_name, Arg&& arg, Args&&... args);

  template <typename Arg>
  void save_error_file(Arg&& arg);

  template <typename Arg>
  void save_histogram(Arg&& arg);

  template <typename Arg>
  void save_controller_count(Arg&& arg);

  template <typename Arg, typename... Args>
  void plot(std::string title,
				    std::string xlabel,
				    std::string ylabel,
				    Arg arg, Arg argv, Args... args);

  void init_dataset_directory();

  std::vector<State<simulator_t>> st_vec() const;

  std::vector<Actions::Action> at_vec() const;

  std::vector<State<simulator_t>> st_1_vec() const;
  std::vector<Actions::Action> at_1_vec() const;

  std::vector<State<simulator_t>> st_2_vec() const;

  arma::mat dataset() const;

	arma::mat test_features() const;
	arma::mat test_labels() const;
	arma::mat train_features() const;
	arma::mat train_labels() const;
	arma::mat trainset() const;
	arma::mat testset() const;

private:
  arma::mat dataset_;

  arma::mat trainset_;
  arma::mat testset_;

  arma::mat train_features_;
  arma::mat train_labels_;

  arma::mat test_features_;
  arma::mat test_labels_;

  double ratio_ = 0.1;

  std::string dataset_file_name_;
  std::string error_file_name_;
  std::string result_file_name_;
  std::string count_file_name_;
  std::string histogram_file_name_;

  std::vector<State<simulator_t>> st_vec_;
  std::vector<Actions::Action> at_vec_;

  std::vector<State<simulator_t>> st_1_vec_;
  std::vector<Actions::Action> at_1_vec_;

  std::vector<State<simulator_t>> st_2_vec_;
};

# include "data_set.hxx"
