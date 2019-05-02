# include "q_learning.hh"

template <class flight_controller_t>
Q_learning<flight_controller_t>::
Q_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	   float speed,
	   std::shared_ptr<Gazebo> gzs,
	   DataSet data_set,
	   bool train)
  :count_(0),
   decay_rate_(0.01),
   discount_rate_(0.95),
   distribution_(0.0, 1.0),
   distribution_int_(0, 3),
   episode_(0),
   epsilon_(1.0),
   learning_rate_(0.3),
   max_episode_(10000),  
   min_epsilon_(0.0),
   rssi_lower_threshold_(1.1),
   rssi_upper_threshold_(0.94)
{
  if(train == true){
    run_episods(iris_x, speed, gzs, data_set);
  }
}
template <class flight_controller_t>
Quadcopter<Gazebo>::Reward
Q_learning<flight_controller_t>::
action_evaluator(lt::triangle<double> old_dist,
		 lt::triangle<double> new_dist)
{
  LogInfo() << "F1 differences: " << std::fabs(old_dist.f1 - new_dist.f1);
  LogInfo() << "F2 differences: " << std::fabs(old_dist.f2 - new_dist.f2);
  
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);
  
  Quadcopter<Gazebo>::Reward reward = Quadcopter<Gazebo>::Reward::very_bad;
  
  if (0.5  > diff_f1 + diff_f2 ) {
    reward = Quadcopter<Gazebo>::Reward::very_good;      
  } else if ( 1.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 0.5 ) {
    reward = Quadcopter<Gazebo>::Reward::good;
  } else if ( 1.5  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.0 ) {
    reward = Quadcopter<Gazebo>::Reward::bad;
  } else if ( 2.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.5 ) {
    reward = Quadcopter<Gazebo>::Reward::very_bad;
  }  
  return reward;
}

/**
 * Returns the accuracy (percentage of correct answers).
 * @param predLabels predicted labels of data points.
 * @param realY real labels (they are double because we usually read them from
 * CSV file that contain many other double values).
 * @return percentage of correct answers.
 */
// double accuracy(arma::Row<size_t> predLabels, arma::Row<size_t> LabelY)
// {
//   // Calculating how many predicted classes are coincide with real labels.
//   size_t success = 0;
//   for (size_t j = 0; j < LabelY.n_cols; j++) {
//     //    std::cout << predLabels(j) << std::endl;
//     //  std::cout << LabelY(j) << std::endl;
//     if (predLabels(j) == LabelY(j)) {
//       ++success;
//     }
//   }
  
//   // Calculating percentage of correctly classified data points.
//   return (double)success / (double)LabelY.n_cols * 100.0;
// }

// arma::Row<size_t> getLabels(const arma::mat& predOut)
// {
//   arma::Row<size_t> pred(predOut.n_cols);
  
//   // Class of a j-th data point is chosen to be the one with maximum value
//   // in j-th column plus 1 (since column's elements are numbered from 0).
//   for (size_t j = 0; j < predOut.n_cols; ++j) {
//     pred(j) = arma::as_scalar(arma::find(
// 					 arma::max(predOut.col(j)) == predOut.col(j), 1)) + 1;
//   }
  
//   return pred;
// }

template <class flight_controller_t>
double Q_learning<flight_controller_t>::deformation_error(lt::triangle<double> old_dist,
				     lt::triangle<double> new_dist)
{
  double error;  
  error = std::sqrt(std::pow((old_dist.f1 - new_dist.f1), 2)  +
		    std::pow((old_dist.f2 - new_dist.f2), 2)  +
		    std::pow((old_dist.f3 - new_dist.f3), 2));
  
  /*  Recalculate the Error between quadcopters  */  
  return error;   
}

template <class flight_controller_t>
lt::positions<double> Q_learning<flight_controller_t>::
get_positions(std::shared_ptr<Gazebo> gzs)
{  
  lt::positions<double> pos;
  pos.leader = gzs->positions().leader;
  pos.f1 = gzs->positions().f1;
  pos.f2 = gzs->positions().f2;

  return pos; 
}

template <class flight_controller_t>
bool Q_learning<flight_controller_t>::
is_signal_in_limits(std::shared_ptr<Gazebo> gzs)
{
  bool ok = false;
  
  float sum_of_neigh_signal = gzs->rssi().lf1() + gzs->rssi().ff() ;
  
  if ( sum_of_neigh_signal < sum_of_neigh_signal* rssi_upper_threshold_ ) {
    if ( sum_of_neigh_signal > sum_of_neigh_signal* rssi_lower_threshold_) {      
      ok = true;      
    }    
  }  
  return ok;  
}

template <class flight_controller_t>
void Q_learning<flight_controller_t>::
move_action(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	    std::string label,
	    float speed,
	    Quadcopter<Gazebo>::Action action)
{
  int quad_number = 0;
  
  if (label == "l"){
    quad_number = 0;
  }
  else if (label == "f1"){
    quad_number = 1;
  }
  else if (label == "f2"){
    quad_number = 2;
  }
        
  if(action == Quadcopter<Gazebo>::Action::left){
    iris_x.at(quad_number)->left(speed);
  }
  else if(action == Quadcopter<Gazebo>::Action::right){ 
    iris_x.at(quad_number)->right(speed);
  }
  else if(action == Quadcopter<Gazebo>::Action::forward){ 
    iris_x.at(quad_number)->forward(speed); 
  }
  else if(action == Quadcopter<Gazebo>::Action::backward){ 
    iris_x.at(quad_number)->backward(speed);
  }  
}

template <class flight_controller_t>
arma::mat Q_learning<flight_controller_t>::
insert_features(std::vector<Quadcopter<Gazebo>::Action> actions)
{  
  arma::mat features;
  
  auto it_state = states_.rbegin();    
  it_state = std::next(it_state, 1);

  for(int i = 0; i < 4; ++i) {
    
    /*  State */
    features << (*it_state).height()
	     << (*it_state).distances().f1
	     << (*it_state).distances().f2
	     << (*it_state).distances().f3
	     << (*it_state).orientation()
      /*  Action  */
	     << mtools_.to_one_hot_encoding(actions.at(i), 4).at(0)
	     << mtools_.to_one_hot_encoding(actions.at(i), 4).at(1)
	     << mtools_.to_one_hot_encoding(actions.at(i), 4).at(2)
	     << mtools_.to_one_hot_encoding(actions.at(i), 4).at(3)
      /*  nextState */
	     << states_.back().height()
	     << states_.back().distances().f1
	     << states_.back().distances().f2
	     << states_.back().distances().f3
	     << states_.back().orientation() << arma::endr;
      
  }
  /*  We need to transpose the matrix, since mlpack is column major */  
  features = features.t();
  
  return features;  
}

template <class flight_controller_t>
Quadcopter<Gazebo>::Action Q_learning<flight_controller_t>::
action_follower(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  Quadcopter<Gazebo>::Action
    action =  Quadcopter<Gazebo>::Action::backward;
  
  if(features(index, 8) == 1) {
    action =  Quadcopter<Gazebo>::Action::forward;
  } else if (features(index, 9) == 1) {
    action =  Quadcopter<Gazebo>::Action::backward;
  } else if (features(index, 10) == 1) {
    action =  Quadcopter<Gazebo>::Action::left;
  } else if (features(index, 11) == 1) {
    action =  Quadcopter<Gazebo>::Action::right;
  }
  return action;
}

/*  Data Set generation */
template <class flight_controller_t>
void Q_learning<flight_controller_t>::
phase_one(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	  float speed,
	  std::shared_ptr<Gazebo> gzs,
	  bool random_leader_action)
{
  /*  Phase One: Construct the dataset */  
  Quadcopter<Gazebo>::Action action_leader ;
  
  std::vector<std::thread> threads;
    
  /* Get the state at time t  */
  Quadcopter<Gazebo>::State state(gzs);
  states_.push_back(state);
  
  if ( random_leader_action == true){
    
    action_leader = randomize_action() ;
    saved_leader_action_ = action_leader;
    //    LogInfo() << "Random action chosen: " << action_leader ;    
  } else {
    action_leader = saved_leader_action_;    
  }
  
  action_follower_.push_back(randomize_action());
  
  /*  Threading QuadCopter */    
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(iris_x, "l" , speed, action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(iris_x, "f1" , speed, action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(iris_x, "f2", speed, action_follower_.back());
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  
  for(auto& thread : threads) {
    thread.join();
  }
  
  /* We need to wait until the quadcopters finish their actions */  
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
  /* Get the next state at time t + 1  */
  Quadcopter<Gazebo>::State nextState(gzs);
  states_.push_back(nextState);

  return ;  
}

template <class flight_controller_t>
std::vector<arma::uword> Q_learning<flight_controller_t>::
index_of_highest_values(arma::mat matrix)   
{  
  std::vector<arma::uword> index;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    index.push_back( arma::index_max(matrix.row(i)) );
  }
  return index;
}

template <class flight_controller_t>
void Q_learning<flight_controller_t>::
phase_two(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	  float speed,                                             
	  std::shared_ptr<Gazebo> gzs,                             
	  bool random_leader_action)
{    
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> model;
  
  mlpack::data::Load("model.xml", "model", model);
  
  /*  we need to pass State ,and nextState, and try possible a */
  Quadcopter<Gazebo>::Action action_leader ;
  
  std::vector<std::thread> threads;
    
  /* Get the state at time t  */
  Quadcopter<Gazebo>::State state(gzs);
  states_.push_back(state);
  
  if ( random_leader_action == true){    
    action_leader = randomize_action() ;
    saved_leader_action_ = action_leader;
  } else {
    action_leader = saved_leader_action_;    
  }
  
  /*  Threading QuadCopter */    
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(iris_x, "l" , speed, action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(iris_x, "f1" , speed, action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));

  /* We need to wait until the quadcopters finish their actions */  
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  
  /* Get the next state at time t + 1  */
  Quadcopter<Gazebo>::State nextState(gzs);
  states_.push_back(nextState);
    
  /*  we need to predict the action for the follower using h(S)*/

  /*  Extract state and push it into the model with several H */
  /*  take the highest value given back by the model */

   
  std::vector<Quadcopter<Gazebo>::Action> possible_action ;

  possible_action.push_back(Quadcopter<Gazebo>::Action::forward);
  possible_action.push_back(Quadcopter<Gazebo>::Action::backward);
  possible_action.push_back(Quadcopter<Gazebo>::Action::left);
  possible_action.push_back(Quadcopter<Gazebo>::Action::right);
  
  arma::mat features = insert_features(possible_action);

  LogInfo() << features;
  
  arma::mat label;    

  model.Predict(features, label);
  
  /*  transpose to the original format */
  features = features.t();
  label = label.t();

  LogInfo() << label;
    
  std::vector<arma::uword> index = index_of_highest_values(label);
       
  arma::uword matrix_row_index = mtools_.index_of_smallest_value(index);
  
  /*  Get the action now !! */
  
  
  Quadcopter<Gazebo>::Action action_for_follower =
    action_follower(features, matrix_row_index);
  
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(iris_x, "f2", speed, action_for_follower);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  
  for(auto& thread : threads) {
    thread.join();
  }
    
  return ;    
}

/* Change action to enum in quadcopter */
template <class flight_controller_t>
Quadcopter<Gazebo>::Action Q_learning<flight_controller_t>::
randomize_action()
{  
  int random_action = distribution_int_(generator_);
  
  LogInfo() << "Random: " << random_action ;

  Quadcopter<Gazebo>::Action action
    = Quadcopter<Gazebo>::Action::forward ; 
  
  if( random_action == 0){
    action = Quadcopter<Gazebo>::Action::forward ;
  }
  else if(random_action == 1){
    action = Quadcopter<Gazebo>::Action::backward ;
  }    
  else if(random_action == 2){
    action = Quadcopter<Gazebo>::Action::left ;
  }
  else if (random_action == 3){
    action = Quadcopter<Gazebo>::Action::right ;   
  }
   return action;
}

template <class flight_controller_t>
void Q_learning<flight_controller_t>::
run_episods(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	    float speed,
	    std::shared_ptr<Gazebo> gzs,
	    DataSet data_set)
{      
  std::vector<lt::position<double>> distance;
  
  lt::positions<double> original_positions = get_positions(gzs);
  
  LogInfo() << "Starting positions : " << original_positions ;
  
  lt::triangle<double> original_triangle = mtools_.triangle_side(original_positions);

  f3_side_.push_back(original_triangle);
  
  for (episode_ = 0; episode_ < max_episode_; ++episode_){
    
    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads: 
     * Arm + takeoff + offboard mode + moving + land 
     * Then: repeat each episode.
     */

    LogInfo() << "Episode : " << episode_ ;            

    /* 
     * Step phase, where we update the q table each step
     * Each episode has a fixed step number.
     * Note: at the end of each episode, we re-intilize everything   
     */
    
    /*  Think How we can use threads here */

    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool arm;
    bool stop_episode = false;    
    for (auto it : iris_x){
      arm = it->arm();
      if(!arm)
	stop_episode = true;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool takeoff;
    for (auto it : iris_x){
      takeoff = it->takeoff();
      if(!takeoff)
	stop_episode = true;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    /*  Setting up speed is important to switch the mode */
    for (auto it : iris_x){
      it->init_speed();
    }
    
    /*  Switch to offboard mode, Allow the control */
    for(auto it : iris_x){
      it->start_offboard_mode();
    }
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));  
    
    /*  Start the First phase, in 3 Steps */
             
    if (!stop_episode) {
      
      count_ = 0 ;
      
      std::vector<lt::triangle<double>> new_triangle;
      
      while (count_ < 3) {

	Quadcopter<Gazebo>::Reward reward =
	  Quadcopter<Gazebo>::Reward::very_bad;
	
	/*  if the follower is has executed a good action we need to
	    re-discover the other action in this loop until we get a 0
	    reward, the other actions are related to exploration and
	    exploitation note that */
	/*  Test if the signal is good, if yes continue with the same
	    action for leader */
	
	if (count_ == 0 ) {
	  phase_one(iris_x, speed, gzs, true);	  
	} else {
	  phase_one(iris_x, speed, gzs, false);      
	}
	
	lt::positions<double> new_positions = get_positions(gzs);
	
	LogInfo() << "New positions : " << new_positions ;
  	
	/*  Get the distance between the TL TF, and FF TF  at time t*/
	new_triangle.push_back(mtools_.triangle_side(new_positions));
	
	/*  Keep a copy of the new distance between all of them */
	f3_side_.push_back(mtools_.triangle_side(new_positions));
	
	/*Calculate the noise over the entire trainning session
	  This will allow to refine exactly the good action */	
	
	double noise = mtools_.gaussian_noise(f3_side_,
					      drift_f3_);
	
	LogInfo() << "Noise: " << noise ;
	
	/* Calculate the error compare to the starting point */
	/* Compare with the original at start */
	if (count_ == 0 ) {
	  reward = action_evaluator(original_triangle,
				    new_triangle.at(0));
	  
	  /*  move it outside of the while loop */
	  if (mtools_.is_triangle(new_triangle.at(0)) == false) {
	    break;
	  }
	  
	} else if (count_ == 1 ) {
	  reward = action_evaluator(new_triangle.at(0),
				    new_triangle.at(1));
	  
	  /*  move it outside of the while loop */
	  if (mtools_.is_triangle(new_triangle.at(1)) == false) {
	    break;
	  }
	  
	} else if (count_ == 2 ) {
	  reward = action_evaluator(new_triangle.at(1),
				    new_triangle.at(2));
	  
	  /*  move it outside of the while loop */
	  if (mtools_.is_triangle(new_triangle.at(1)) == false) {
	    break;
	  }	  	  
	}
	
	/*  1- problem logging the actions the next action is not
	    registered before being logged */
	
	auto it_state = states_.rbegin();
	auto it_action = action_follower_.rbegin();
	
	it_state = std::next(it_state, 1);
	it_action = std::next(it_action, 1);
	
	Quadcopter<Gazebo>::State sp(gzs);
	
	data_set.save_csv_data_set(sp.create_printer_struct(*it_state),
				   mtools_.to_one_hot_encoding(action_follower_.back(), 4),
				   sp.create_printer_struct(states_.back()),
				   mtools_.to_one_hot_encoding(reward, 4)
				   );
	
	//	LogInfo() << "reward: " << reward ;
	
	//reduce epsilon as we explore more each episode
	epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode_); 
	
	LogInfo() << "Epsilon: " << epsilon_ ;	
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	++count_;
    }
    }

    for (auto it: iris_x)
      it->land();
    
    std::this_thread::sleep_for(std::chrono::seconds(6));
    
    gzs->reset_models();
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
    
    /*BIAS accelerometer problem after resetting the models*/
      
    /*  The only possible solution was to change the upper limit value
     * for the bias inside thee code of te firmware direclty. The
     * solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they propose
     * to increase the value of COM_ARM_EKF_AB. Note that, the default
     * value is 0.00024 I have increased it to 0.00054 which is very
     * high to the usual stadard. Other wise there is no way to do the
     * simulation. Remember, the reboot() function in the action class
     * is not implemented at the time of writing this comment, and
     * maybe it will never be implemented as it is quite complicated
     * to reboot the px4 software from the simulator. I understand
     * this choice, we need to leave a big sleep_for after resetting
     * the quadcopters, that is going to helpe resetting the
     * accelerometer values without any problems!
     */
    
    /*  I have quite tested a lot of different solution, if I am going
     * to find a better one, I will replace it directly. */      
  }
}
