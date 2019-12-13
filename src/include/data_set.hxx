#pragma once

# include "data_set.hh"

template <class simulator_t>
DataSet<simulator_t>::DataSet()
{}

template <class simulator_t>
void DataSet<simulator_t>::read_dataset_file(std::string file_name)
{
  dataset_.load(file_name, arma::csv_ascii);

    // We need to create a state from the first 3 column from a matrix
  // It is much better to parse data set line by line into state, actions, nextstate , etc
  /*  Then all the parsed state and actions can be agglomerated into matries.
   Then it should be easy to read each line of these matrices since they are seperated */

  // /*  Get the indices for state, actions etc... */
  /*  This should be the standard method to parse the dataset */
  // for (int i = 0; i < dataset_.n_cols; ++i) {
  //   if ((dataset_(0,i) ==  and (dataset_(0,i) == 0  or dataset_(0,i) == 1)) {
      
  //   }
  // }
  
  arma::mat st_mat = dataset_(arma::span(0 ,dataset_.n_rows -1 ), arma::span(0, 2));
  arma::mat at_mat = dataset_(arma::span(0 ,dataset_.n_rows -1 ), arma::span(3, 9));
  arma::mat st_1_mat = dataset_(arma::span(0 ,dataset_.n_rows -1 ), arma::span(10, 12));
  arma::mat at_1_mat = dataset_(arma::span(0 ,dataset_.n_rows -1 ), arma::span(13, 19));
  arma::mat st_2_mat = dataset_(arma::span(0 ,dataset_.n_rows -1 ), arma::span(20, 22));

  State<simulator_t> state;
  Actions action;
  st_vec_ = state.StateConstructor(st_mat);
  at_vec_ = action.ActionConstructor(at_mat);
  st_1_vec_ = state.StateConstructor(st_1_mat);
  at_1_vec_ = action.ActionConstructor(at_1_mat);
  st_2_vec_ = state.StateConstructor(st_2_mat);  
}

template <class simulator_t>
arma::mat DataSet<simulator_t>::dataset() const
{ return dataset_; }

template <class simulator_t>
std::vector<State<simulator_t>> DataSet<simulator_t>::st_vec() const
{ return st_vec_; }

template <class simulator_t>
std::vector<Actions::Action> DataSet<simulator_t>::at_vec() const
{ return at_vec_; }

template <class simulator_t>
std::vector<State<simulator_t>> DataSet<simulator_t>::st_1_vec() const
{ return st_1_vec_; }

template <class simulator_t>
std::vector<Actions::Action> DataSet<simulator_t>::at_1_vec() const
{ return at_1_vec_; }

template <class simulator_t>
std::vector<State<simulator_t>> DataSet<simulator_t>::st_2_vec() const
{ return st_2_vec_; }

template <class simulator_t>
void DataSet<simulator_t>::init_dataset_directory()
{
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();

  std::stringstream date_stream, time_stream;

  date_stream << now.date();
  time_stream << now.time_of_day();

  std::experimental::filesystem::create_directory("../dataset");
  std::experimental::filesystem::create_directory("../dataset/" + date_stream.str());

  dataset_file_name_=
    "../dataset/" + date_stream.str() + "/" + time_stream.str();

  error_file_name_=
    "../dataset/" + date_stream.str() + "/error" + time_stream.str();

  histogram_file_name_=
    "../dataset/" + date_stream.str() + "/histogram" + time_stream.str();

  count_file_name_=
    "../dataset/" + date_stream.str() + "/count" + time_stream.str();

  result_file_name_ =
    "../dataset/" + date_stream.str() + "/figure" + time_stream.str();
}

template <class simulator_t>
template <typename Arg, typename... Args>
void DataSet<simulator_t>::write_data_set_file(std::ofstream& file, Arg&& arg, Args&&... args)
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

template<class simulator_t>
template <typename Arg, typename... Args>
void DataSet<simulator_t>::save_csv_data_set(Arg&& arg, Args&&... args)
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

template<class simulator_t>
template <typename Arg, typename... Args>
void DataSet<simulator_t>::save_csv_data_set_2_file(std::string file_name, Arg&& arg, Args&&... args)
{
  std::ofstream file;
  file.open(dataset_file_name_ + "_" + file_name,
	    std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file <<","<< std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<class simulator_t>
template <typename Arg>
void DataSet<simulator_t>::save_error_file(Arg&& arg)
{
  std::ofstream file;
  file.open(error_file_name_,
	    std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) <<"\n";

  file.flush();
  file.close();
}

template<class simulator_t>
template <typename Arg>
void DataSet<simulator_t>::save_histogram(Arg&& arg)
{
  std::ofstream file;
  file.open(histogram_file_name_,
	    std::ios::out);

  for (auto& [key, value]: arg) {
    file << key << " " << value << "\n";
  }
  file.flush();
  file.close();
}

template<class simulator_t>
template <typename Arg >
void DataSet<simulator_t>::save_controller_count(Arg&& arg)
{
  std::ofstream file;
  file.open(count_file_name_,
	    std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) <<"\n";

  file.flush();
  file.close();
}

template<class simulator_t>
template <typename Arg, typename... Args>
void DataSet<simulator_t>::plot(std::string title,
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
