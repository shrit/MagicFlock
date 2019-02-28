#ifndef _DATA_SET_
#define _DATA_SET_


# include <algorithm>
# include <chrono> 
# include <ctime>
# include <fstream>
# include <iomanip>
# include <iostream>
# include <iterator>
# include <sstream>
# include <string>
# include <vector>
# include <unordered_map>

/*  Boost library include */
# include <boost/algorithm/string.hpp>

/*  local defined include */
# include "global.hh"



namespace lt = local_types;


class DataSet
{
public:
  
  DataSet();
    
  void read_data_set_file(std::string file_name);
  template <typename A, typename B, typename C>  
  void write_data_set_file(std::ofstream& file, A a, B b, C c);
  
  template <typename A, typename B, typename C>  
  void write_csv_data_set_file(std::ofstream& file, A a, B b, C c);
  
  template <typename A, typename B>  
  void read_map_file(std::string file_name, std::unordered_map <A, B>& map_);
  
  template <typename A, typename B>  
  void write_map_file(std::string file_name, std::unordered_map <A, B> map_);
  
  std::vector<lt::rssi<double>>  rssi_vector() const;
  std::vector<int>  action_vector() const;
  std::vector<lt::error<double>> error_vector() const;
  
  std::vector<std::vector<double>> data_set() const;
  
  
private:
  
  int line_number_;

  std::vector<std::vector<double>> data_set_;  
    
};
# include "data_set_impl.hh"


#endif
