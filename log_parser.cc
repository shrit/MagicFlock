/*  C++ STL include */

# include <fstream>
# include <iostream>
# include <iterator>
# include <string>
# include <vector>
# include <algorithm>

/*  Boost library include */
# include <boost/algorithm/string.hpp>

/*  local defined include */
# include "global.hh"


class DataSet
{
public:
  
  DataSet();
  

  


void read_data_set_file(std::string file_name,
		    std::vector<rssi<double>>& rssi_,
		    std::vector<int>& action_,
		    std::vector<error<double>>& error_)
{
  std::string line;
  std::ifstream fs;
  fs.open(file_name);

  while (std::getline(fs, line))
    {
      /*   
	   parse each line by itself and put the result into the data
	   structure. Find all the delimiter and replace them with
	   space then split the values from the string into a vector	   
      */
      line_number++;
      
      line = line.substr(10);
      std::vector<std::string> values;
      std::string::size_type sz;
      std::replace_if(line.begin(), line.end(), boost::is_any_of(",[]") , ' ');
      boost::split(values, line, boost::is_any_of(" "));

      rssi r;
      
      r.lf1 = std::stod(values.at(1), &sz);
      r.lf2 = std::stod(values.at(3), &sz);
      r.ff  = std::stod(values.at(5), &sz);

      rssi_.push_back(r);
      
      action_.push_back(std::stod(values.at(7), &sz));

      error e;

      e.x = std::stod(values.at(9), &sz);
      e.y = std::stod(values.at(11), &sz);

      error_push_back(e);

    } 
}
  
private:
  
  int line_number_;
  std::string file_name;
  std::vector<rssi<double>>& rssi_;
  std::vector<int>& action_;
  std::vector<error<double>>& error_;
    
};




int main(int argc, char* argv[])
{

  std::vector<rssi<double>> rssi;
  std::vector<int> action;
  std::vector<error<double>> error;
  
  read_data_set_file("data_sample", rssi, action, error);

  std::cout << "RSSI: " << rssi << std::endl;

  std::cout << "Action: " << action << std::endl;
  
  std::cout << "Error: " << error << std::endl;
  
  
}

