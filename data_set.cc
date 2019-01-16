# include "data_set.h"

DataSet::DataSet();

DataSet::read_data_set_file(std::string file_name)
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
      line_number_++;
      
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
  fs.close();
  
}

template <typename A, typename B, typename C>
void write_data_set_file(std::ofstream& file, A a, B b, C c)
{
  std::stringstream time;
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  
  time << std::put_time(std::localtime(&in_time_t), "%H:%M:%S");

  file << "[" << time.str() << "]" << a <<"["<< b <<"]"<< c << "\n";
  file.flush();  
}

std::vector<lt::rssi<double>>  DataSet::rssi_vector() const
{  return rssi_; }

std::vector<int>  DataSet::action_vector() const
{return action_;}

std::vector<lt::error<double>> DataSet::error_vector() const
{ return error_; }  
