# ifndef Q_LEARNING_HH_
# define Q_LEARNING_HH_

/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <iostream>
# include <numeric>
# include <random>
# include <thread>
# include <vector>
# include <unordered_map>

/*  Armadillo includes  */
# include <armadillo>

/* Quadcopter controller includes  */
# include "../px4_device.hh"
# include "../global.hh"
# include "../gazebo.hh"
# include "../data_set.hh"
# include "../log.hh"


class Q_learning
{
  
public:
  
  Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
	     float speed,
	     std::shared_ptr<Gazebo> gzs,
	     DataSet data_set,
	     bool train);

  bool action_evaluator(lt::triangle<double> old_dist,
  			lt::triangle<double> new_dist,
			double noise);
  
  int cantor_pairing(int x, int y);
  
  double deformation_error(lt::triangle<double>  old_dist,
			   lt::triangle<double>  new_dist);
  double gaussian_noise(std::vector<lt::triangle<double>> ideal_dist);
  
  lt::positions<double> get_positions(std::shared_ptr<Gazebo> gzs);
    
  bool is_signal_in_limits(std::shared_ptr<Gazebo> gzs);  

  bool is_triangle(lt::triangle<double> t);

  void move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   std::string label,
		   float speed,
		   lt::action<bool> action);  
  
  void phase_one(std::vector<std::shared_ptr<Px4Device>> iris_x,
		 float speed,
		 std::shared_ptr<Gazebo> gzs,
		 bool random_leader_action);
  
  void phase_two();

  arma::uword qtable_action(arma::mat qtable, arma::uword state);
  lt::action<bool> qtable_action_binary(arma::mat qtable, arma::uword state);

  double qtable_value(arma::mat q_table, arma::uword state);
  
  arma::uword qtable_state(std::shared_ptr<Gazebo> gzs, bool value);

  arma::uword qtable_state_from_map(std::shared_ptr<Gazebo> gzs,
			    std::unordered_map<int, int> map);

  lt::action<bool> randomize_action();
  
  void run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   std::shared_ptr<Gazebo> gzs,
		   DataSet data_set);
      
  lt::triangle<double> triangle_side(lt::positions<double> pos);
  double variance(double mean);
  void update_qtable(int reward);
  
private:

  int count_;
  float decay_rate_ ;   
  float discount_rate_ ;
  std::uniform_real_distribution<> distribution_;
  std::uniform_int_distribution<> distribution_int_;
  int episode_ ;
  float epsilon_ ;
  float learning_rate_ ;
  std::vector<float> lower_threshold_;
  int max_episode_ ;  
  int max_step_ ;
  float min_epsilon_ ;  
  arma::mat  qtable_;
  float rssi_lower_threshold_;
  float rssi_upper_threshold_;
  std::vector<float> upper_threshold_;
  
  std::vector<lt::rssi<double>>   states_, new_state_;
    
  std::vector<double> rewards_;
  
  std::default_random_engine generator_;
  
  std::unordered_map<int, int> signal_map_;
    
  arma::uword index_, new_index_;
      
  std::vector<lt::action<bool>> action_follower_ ;
  
  lt::action<bool> saved_leader_action_;

  std::vector<lt::triangle<double>> f3_side_;
  std::vector<double> diff_f3_;
  int random_action_follower_;
  
};
  
#endif
