# include <fstream>
# include <iostream>
# include <iterator>
# include <string>
# include <vector>
# include <algorithm>
# include <boost/algorithm/string.hpp>

template <typename T> 
struct rssi {
  T lf1;
    T lf2;
  T ff;
};


template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, " "));
    out << "]";
  }
  return out;
}


template <typename T>
std::ostream& operator<< (std::ostream& out, const rssi<T>& r)
{
  out << "["<< r.lf1 <<", " << r.lf2 <<", " << r.ff <<"]";
  return out;
}


void parse_log_file(std::string file_name, rssi<double>& rssi, int& action, std::vector<double>& error)
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
      line = line.substr(10);
      std::vector<std::string> values;
      std::string::size_type sz;
      std::replace_if(line.begin(), line.end(), boost::is_any_of(",[]") , ' ');
      boost::split(values, line, boost::is_any_of(" "));
    
      rssi.lf1 = std::stod(values.at(1), &sz);
      rssi.lf2 = std::stod(values.at(3), &sz);
      rssi.ff  = std::stod(values.at(5), &sz);
      action   = std::stod(values.at(7), &sz);
      error.push_back(std::stod(values.at(9), &sz));
      error.push_back(std::stod(values.at(11), &sz));
    } 
}

int main(int argc, char* argv[])
{
  rssi<double> rssi;
  int action = 0;
  std::vector<double> error;
  
  parse_log_file("data_sample", rssi, action, error);

  std::cout << "RSSI: " << rssi << std::endl;

  std::cout << "Action: " << action << std::endl;
  
  std::cout << "Error: " << error << std::endl;
  
  
}

