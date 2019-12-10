# include "include/data_set.hh"

DataSet::DataSet()
{}

void DataSet::read_dataset_file()
{
  dataset_.load(dataset__file, csv_ascii);

    // We need to create a state from the first 3 column from a matrix
  // It is much better to parse data set line by line into state, actions, nextstate , etc
  /*  Then all the parsed state and actions can be agglomerated into matries.
   Then it should be easy to read each line of these matrices since they are seperated */

  /*  Get the indices for state, actions etc... */
  for (int i = 0; i < dataset_.n_cols; ++i) {
    if (dataset_(0,i) ==  and (dataset_(0,i) == 0  or dataset_(0,i) == 1)) {
      
    }
  }
  
  arma::mat st_mat = dataset_(span(0 ,dataset_.n_rows -1 ), span(0,2 ));
  arma::mat at_mat = dataset_(span(0 ,dataset_.n_rows -1 ), span(3, ));
  arma::mat st_1_mat = dataset_(span(0 ,dataset_.n_rows -1 ), span(, ));
  arma::mat at_1_mat = dataset_(span(0 ,dataset_.n_rows -1 ), span(, ));
  arma::mat st_2_mat = dataset_(span(0 ,dataset_.n_rows -1 ), span(, ));
  
  st_vec_ = StateConstructor(st_mat);
  at_vec_ = ActionConstructor(at_ma);
  st_1_vec_ = StateConstructor(st_1_mat);
  at_1_vec_ = ActionConstructor(at_1_mat);
  st_2_vec_ = StateConstructor(st_2_mat);  
}

arma::mat DataSet::dataset() const
{ return dataset_; }

std::vector<State<simulator_t>> st_vec() const
{ return st_vec_; }

std::vector<Actions::Action> at_vec() const
{ return at_vec_; }

std::vector<State<simulator_t>>  st_1_vec() const
{ return st_1_vec_; }

std::vector<Actions::Action> at_1_vec() const
{ return at_1_vec_; }

std::vector<State<simulator_t>> st_2_vec() const
{ return st_2_vec_; }

void DataSet::init_dataset_directory()
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
