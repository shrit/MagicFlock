# include "data_set.h"

DataSet::DataSet() :
  line_number_(0)
{
  
}

void DataSet::read_data_set_file(std::string file_name)
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
	   If the number of matrix element change, we only need to
	   change the two numbers in the following code.
      */
      line_number_++;
      
      line = line.substr(11);
      std::vector<std::string> values;
      std::string::size_type sz;
      std::replace_if(line.begin(), line.end(), boost::is_any_of(",[]") , ' ');
      boost::split(values, line, boost::is_any_of(" "), boost::token_compress_on);
      
      values.resize(6);
      std::vector<double> double_values(values.size());
      
      std::transform(values.begin(), values.end(), double_values.begin(), [](const std::string& val)
      									  {
      									    return std::stod(val);
      									  });      
      data_set_.push_back(double_values);            

    }
  fs.close();
  
}

std::vector<std::vector<double>>  DataSet::data_set() const
{  return data_set_; }
