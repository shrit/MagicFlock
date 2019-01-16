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
struct error {
  T x;
  T y;
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


void parse_log_file(std::string file_name,
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

int main(int argc, char* argv[])
{

  std::vector<rssi<double>> rssi;
  std::vector<int> action;
  std::vector<error<double>> error;
  
  parse_log_file("data_sample", rssi, action, error);

  std::cout << "RSSI: " << rssi << std::endl;

  std::cout << "Action: " << action << std::endl;
  
  std::cout << "Error: " << error << std::endl;
  
  
}

