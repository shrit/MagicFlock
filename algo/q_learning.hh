
/*  Standard C++ includes  */
# include <iostream>
# include <vector>

/*  Boost numeric includes */
# include <boost/numeric/ublas/matrix.hpp>
# include <boost/numeric/ublas/io.hpp>

/* Quadcopter controller includes  */
# include "../controller.hh"

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

using namespace boost::numeric::ublas;
    

class Q_learning
{

public:
  Q_learning()
    :qtable_{4096,5},
     rewards_{}
    {}

  void init();
  void run_episods();
  void move_quads_followers_action(Controllers controllers, int action);
       
private:
  
  matrix<double> qtable_;
  vector<float> rewards_;
  
  Gazebo gazebo_;
  
  Controller controller;
  

 
  
};

}
