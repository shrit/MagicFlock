/*  Standard C++ includes  */
# include <iostream>
# include <vector>
# include <cstdlib>
/* Eigne includes */
# include <Eigen/Dense>



/* Quadcopter controller includes  */
# include "../controller.hh"
# include "../global.hh"

# include "../gazebo.hh"


/*  Parameters for Q-learning algorithm  */

namespace algo{

  namespace q_values{
    
    const int max_episode = 10000;
    const int max_step    = 2;
    const float learning_rate = 0.9;
    const float discount_rate =0.95;
    
    const int epsilon = 1;
    const int min_epsilon = 0;
    const float decay_rate  = 0.01;
    
  }
  
  
  namespace Eigen{
    template<class Matrix>
    void write_binary(const char* filename, const Matrix& matrix){
      std::ofstream out(filename,ios::out | ios::binary | ios::trunc);
      typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
      out.write((char*) (&rows), sizeof(typename Matrix::Index));
      out.write((char*) (&cols), sizeof(typename Matrix::Index));
      out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
      out.close();
    }
    template<class Matrix>
    void read_binary(const char* filename, Matrix& matrix){
      std::ifstream in(filename,ios::in | std::ios::binary);
      typename Matrix::Index rows=0, cols=0;
      in.read((char*) (&rows),sizeof(typename Matrix::Index));
      in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
    }
  } // Eigen::
  
  
  
  
  class Q_learning
  {
    
  public:
    Q_learning();
    
    void init();
    void run_episods(std::vector<std::shared_ptr<Controller>> controllers);
    void move_quads_followers_action(std::vector<std::shared_ptr<Controller>> controllers,
				     int action);
       
  private:

    Eigne::MatrixXd  qtable_;
    Eigne::MatrixXd  states_, new_state_;

    Eigne::VectorXd  rewards_;
    
    Gazebo gazebo_;  
  
    Controller controller_;
  
  
 
  
  };

}
