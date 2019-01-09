# ifndef Q_LEARNING_HH_
# define Q_LEARNING_HH_


/*  Standard C++ includes  */
# include <iostream>
# include <vector>
# include <cstdlib>
# include <cmath>
# include <algorithm>
# include <chrono>
# include <thread>

/* Quadcopter controller includes  */
# include "../px4_device.hh"
# include "../global.hh"
# include "../gazebo.hh"


/*  Parameters for Q-learning algorithm  */

//namespace algo{


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
  
namespace Data{
  template<class Matrix>
  void write_binary(const char* filename, const Matrix& matrix){
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
  }
  template<class Matrix>
  void read_binary(const char* filename, Matrix& matrix){
    std::ifstream in(filename, std::ios::in | std::ios::binary);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows), sizeof(typename Matrix::Index));
    in.read((char*) (&cols), sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
  }
} 
        
class Q_learning
{
    
public:

  Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
	     float speed,
	     std::shared_ptr<Gazebo> gzs);
    
  void init();

  int get_action(std::vector<std::vector<double>>  qtable , double state);

  double get_state_index(lt::rssi<double> signal, lt::rssi<double> original_signal);
  
  void move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   int action,
		   int quad_number);  
  
  void run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
		   float speed,
		   std::shared_ptr<Gazebo> gzs);

       
private:
    
  std::vector<std::vector<double>>  qtable_;
    
  lt::rssi<double>   original_signal_, states_, new_state_;
    
  std::vector<double> rewards_;
  
  int max_episode_ ;       
  int max_step_ ;            
  int epsilon_ ;        
  int min_epsilon_ ;    
  float decay_rate_ ;   
  float learning_rate_ ;
  float discount_rate_ ;
     
};

  //}

#endif
