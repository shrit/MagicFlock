#pragma once

template<typename ModelType>
void
DataSet::save_model(ModelType&& model, std::string model_name)
{
  mlpack::data::Save(
    model_file_name_, model_name, std::forward<ModelType>(model), false);
}

template<typename Arg>
void
DataSet::save_state(std::string file_name, Arg&& arg)
{
  std::ofstream file;
  file.open(state_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);

  file << "\n";
  file.flush();
  file.close();
}

template<typename Arg, typename... Args>
void
DataSet::save_evaluation(Arg&& arg, Args&&... args)
{
  std::ofstream file;
  file.open(evaluate_file_name_, std::ios::out);

  file << std::forward<Arg>(arg);
  ((file << std::forward<Args>(args)), ...);
  file << "\n";
  file.flush();
  file.close();
}

template<typename Arg, typename... Args>
void
DataSet::save_csv_dataset(Arg&& arg, Args&&... args)
{
  std::ofstream file;
  file.open(dataset_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file << "," << std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<typename Arg, typename... Args>
void
DataSet::save_csv_dataset_2_file(std::string file_name,
                                 Arg&& arg,
                                 Args&&... args)
{
  std::ofstream file;
  file.open(dataset_file_name_ + "_" + file_name + ".csv",
            std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file << "," << std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<typename Arg, typename... Args>
void
DataSet::save_error_file(std::string file_name, Arg&& arg, Args&&... args)
{
  std::ofstream file;
  file.open(error_file_name_ + "_" + file_name + ".csv",
            std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg);
  ((file << "," << std::forward<Args>(args)), ...);

  file << "\n";
  file.flush();
  file.close();
}

template<typename Arg>
void
DataSet::save_histogram(Arg&& arg)
{
  std::ofstream file;
  file.open(histogram_file_name_, std::ios::out);

  for (auto& [key, value] : arg) {
    file << key << " " << value << "\n";
  }
  file.flush();
  file.close();
}

template<typename Arg>
void
DataSet::save_controller_count(Arg&& arg)
{
  std::ofstream file;
  file.open(count_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) << "\n";

  file.flush();
  file.close();
}

template<typename Arg>
void
DataSet::save_episodes(Arg&& arg)
{
  std::ofstream file;
  file.open(episodes_file_name_, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) << "\n";

  file.flush();
  file.close();
}

template<typename Arg>
void
DataSet::save_actions(std::string file_name, Arg&& arg)
{
  std::ofstream file;
  file.open(action_file_name_ + "_" + file_name, std::ios::out | std::ios::app);

  file << std::forward<Arg>(arg) << "\n";

  file.flush();
  file.close();
}

template<typename StateType>
void
DataSet::plot_distance_to_neighbor(int neighbor_id,
                                   std::string title,
                                   std::string xlabel,
                                   std::string ylabel,
                                   std::string file_name,
                                   std::string type,
                                   StateType states)
{
}

template<typename HistoType>
void
DataSet::plot_histogram(int neighbor_id,
                        std::string title,
                        std::string xlabel,
                        std::string ylabel,
                        std::string file_name,
                        std::string type,
                        HistoType histogram)
{
}

template<typename xType, typename yType>
void
DataSet::plot(std::string title,
              std::string xlabel,
              std::string ylabel,
              std::string file_name,
              std::string type,
              xType x,
              yType y)
{}


