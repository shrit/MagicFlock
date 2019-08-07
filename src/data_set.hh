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

# include <armadillo>

/*  local defined include */
# include "global.hh"

# include "../third_party/gnuplot/plot.h"

namespace lt = local_types;

class DataSet {

public:

  DataSet();

  void read_data_set_file(std::string file_name);

  template <typename Arg, typename... Args>
  void write_data_set_file(std::ofstream& file, Arg&& arg, Args&&... args);

  template <typename Arg, typename... Args>
  void save_csv_data_set(Arg&& arg, Args&&... args);

  template <typename Arg>
  void save_error_file(Arg&& arg);

  template <typename Arg>
  void save_histogram(Arg&& arg);

  template <typename Arg>
  void save_count_file(Arg&& arg);

  template <typename Arg, typename... Args>
  void plot(std::string title,
	    std::string xlabel,
	    std::string ylabel,
	    Arg arg, Arg argv, Args... args);

  void init_dataset_directory();

  std::vector<lt::rssi<double>>  rssi_vector() const;
  std::vector<int>  action_vector() const;
  std::vector<lt::error<double>> error_vector() const;
  std::vector<std::vector<double>> data_set() const;

private:

  int line_number_;

  std::string dataset_file_name_;
  std::string error_file_name_;
  std::string result_file_name_;
  std::string count_file_name_;
  std::string histogram_file_name_;

  std::vector<std::vector<double>> data_set_;

};

# include "data_set.hxx"
