#pragma once

# include "data_set.hh"

template <typename Arg, typename... Args>
void DataSet::write_data_set_file(std::ofstream& file, Arg&& arg, Args&&... args)
{
  std::stringstream time;
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  
  time << std::put_time(std::localtime(&in_time_t), "%H:%M:%S");

  
  file << "[" << time.str() << "]" << std::forward<Arg>(arg) <<"[" ;
  ((file << std::forward<Args>(args)), ...);
  file << "]" << "\n";
  file.flush();  
}

template <typename Arg, typename... Args>  
void DataSet::save_csv_data_set(Arg&& arg, Args&&... args)
{  
  std::ofstream file;
  file.open(dataset_file_name_,
	    std::ios::out | std::ios::app);
  
  file << std::forward<Arg>(arg);
  ((file <<","<< std::forward<Args>(args)), ...);
  
  file << "\n";    
  file.flush();  
  file.close();
}

template <typename Arg>  
void DataSet::save_error_file(Arg&& arg)
{
  std::ofstream file;
  file.open(error_file_name_,
	    std::ios::out | std::ios::app);
  
  file << std::forward<Arg>(arg) <<"\n";
  
  file.flush();
  file.close();  
}

template <typename A, typename B>  
void DataSet::read_map_file(std::string file_name, std::unordered_map<A, B>& map_)
{
  std::string line;
  std::ifstream fs;
  fs.open(file_name);
  
  while (std::getline(fs, line)) {
    
    A key;
    B value;
    std::stringstream ss(line);
    if (ss >> key){
      if(ss >> value){	    
	map_[key] = value;
      }        
    }
  }
  
  fs.close();
}

template <typename Arg, typename... Args>
void DataSet::plot(std::string title,
		   std::string xlabel,
		   std::string ylabel,		   
		   Arg arg, Arg argv, Args... args)
{

  /*  define here if the args are equal to the std vector 
   Use variadic template for  x, y */

  /*  Gnuplot Config */
  plotcpp::Plot plt;
  plt.SetTerminal("svg");
  plt.SetOutput(result_file_name_);
  plt.SetTitle(title);
  plt.SetXLabel(xlabel);
  plt.SetYLabel(ylabel);
  plt.SetAutoscale();

  /*  Need to look into it... */
  // plt.Draw2D(Points(arg.begin(), arg.end(), argv.begin(), "Reward"),
  // 	     Points(x_zero.begin(), x_zero.end(), y_zero.begin(), "No reward"));

  plt.Flush();
      
}

