# ifndef Q_LEARNING_HH_
# define Q_LEARNING_HH_


/*  Standard C++ includes  */
# include <algorithm>
# include <chrono>
# include <cmath>
# include <iostream>
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
# include "../data_set.h"
# include "../log.hh"
# include "../global_include.h"

template <typename T>
std::ostream& operator<< (std::ostream& out,
			  const Telemetry::PositionVelocityNED& p)
{					   
  out << "[ "
      << p.position.north_m <<", "
      << p.position.east_m <<", "
      << p.position.down_m <<
    "]"<<"\n";
  return out;
}
  
        
class Q_learning
{
    
public:

  Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
	     float speed,
	     std::shared_ptr<Gazebo> gzs,
	     DataSet data_set,
	     bool train);
   
  int cantor_pairing(int x, int y);
  
  double qtable_action(arma::mat qtable , arma::uword state);

  arma::uword qtable_state(std::shared_ptr<Gazebo> gzs, bool value);

  arma::uword qtable_state_from_map(std::shared_ptr<Gazebo> gzs,
			    std::unordered_map<int, int> map);

  bool is_signal_in_limits(std::shared_ptr<Gazebo> gzs);
  
  void move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   int action,
		   int quad_number);  
  
  lt::triangle<double> triangle_side(std::vector<lt::position<double>> pos);

  bool is_triangle(lt::triangle<double> t);
  
  double deformation_error(std::vector<lt::position<double>> pos);

  void explore_actions(std::vector<std::shared_ptr<Px4Device>> iris_x,
		       float speed);  
  
  void run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   std::shared_ptr<Gazebo> gzs,
		   DataSet data_set);

private:
    
  arma::mat  qtable_;
    
  lt::rssi<double>   new_state_;
    
  std::vector<double> rewards_;
  
  std::default_random_engine generator_;
  std::uniform_real_distribution<> distribution_;
  
  std::uniform_int_distribution<> distribution_int_;

  std::unordered_map<int, int> signal_map_;
  
  int max_episode_ ;
  int episode_ ;
  int max_step_ ;  
  arma::uword index_;
  float epsilon_ ;        
  float min_epsilon_ ;    
  float decay_rate_ ;   
  float learning_rate_ ;
  float discount_rate_ ;
  std::vector<float> upper_threshold_;
  std::vector<float> lower_threshold_;
  float rssi_upper_threshold_;
  float rssi_lower_threshold_;
  
     
};

  //}

#endif
