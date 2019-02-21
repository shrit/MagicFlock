
template <typename A, typename B, typename C>
void DataSet::write_data_set_file(std::ofstream& file, A a, B b, C c)
{
  std::stringstream time;
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  
  time << std::put_time(std::localtime(&in_time_t), "%H:%M:%S");

  file << "[" << time.str() << "]" << a <<"["<< b <<"]"<< c << "\n";
  file.flush();  
}


template <typename A, typename B, typename C>  
void DataSet::write_csv_data_set_file(std::ofstream& file, A a, B b, C c)
{
  file << a <<","<< b <<","<< c << "\n";
  file.flush();    
}


template <typename A, typename B>  
void DataSet::write_map_file(std::string file_name, std::unordered_map <A, B> map_)
{
  std::ofstream file;
  file.open(file_name);
  
  for(auto [key, value] : map_)
    file << key <<" "<< value <<"\n";
  file.flush();
  file.close();
  
}

template <typename A, typename B>  
void DataSet::read_map_file(std::ofstream& file, std::unordered_map <A, B> map_)
{

  A key;
  B value;
  
  while (file >> key >> value)
    map_[key] = value;

}
