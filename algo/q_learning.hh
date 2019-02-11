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
/*  Armadillo includes  */
# include <armadillo>

/* Quadcopter controller includes  */
# include "../px4_device.hh"
# include "../global.hh"
# include "../gazebo.hh"
# include "../log.hh"
# include "../data_set.h"



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
	     DataSet data_set);
    
  void init();

  double get_action(arma::mat qtable , double state);

  double get_state_index(lt::rssi<double> signal, lt::rssi<double> original_signal);
  
  void move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   int action,
		   int quad_number);  
  
  void run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   std::shared_ptr<Gazebo> gzs,
		   DataSet data_set);

       
private:
    
  arma::mat  qtable_;
    
  lt::rssi<double>   original_signal_, states_, new_state_;
    
  std::vector<double> rewards_;

  //  std::random_device rd;
  //  std::mt19937 gen(rd());
  std::default_random_engine generator_;
  std::uniform_real_distribution<> distribution_;

  std::uniform_int_distribution<> distribution_int_;
  
  
  int max_episode_ ;       
  int max_step_ ;            
  float epsilon_ ;        
  float min_epsilon_ ;    
  float decay_rate_ ;   
  float learning_rate_ ;
  float discount_rate_ ;
     
};

  //}

#endif
