#pragma once

# include "supervised_learning.hh"

template <class flight_controller_t,
	  class simulator_t>
Supervised_learning<flight_controller_t, simulator_t>::
Supervised_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
		    std::shared_ptr<simulator_t> gzs)
  :count_(0),
   episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(gzs)),
   swarm_(std::move(iris_x))
{
  data_set_.init_dataset_directory();
}

/* Estimate the features (distances) using propagation model from RSSI */
template <class flight_controller_t,
	  class simulator_t>
arma::mat Supervised_learning<flight_controller_t,
		     simulator_t>::
insert_estimated_features(std::vector<Quadcopter::Action> actions)
{
  arma::mat features;
  auto it_state = states_.rbegin();
  it_state = std::next(it_state, 1);

  arma::rowvec row;
  for (int i = 0; i < 7; ++i) {
    /*  State */
    row << (*it).distances_3D().f1
	<< (*it).distances_3D().f2
	<< (*it).height_difference()
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(0)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(1)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(2)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(3)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(4)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(5)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(6)
	<< states_.back().distances_3D().f1
	<< states_.back().distances_3D().f2
	<< states_.back().height_difference()
      /*  Action encoded as 1, and 0, add 7 times to represent 7 actions */
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(0)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(1)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(2)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(3)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(4)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(5)
    	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(6);

    features.insert_rows(0, row);
  }
  /*  We need to transpose the matrix, since mlpack is column major */
  features = features.t();
  return features;
}

template <class flight_controller_t,
	  class simulator_t>
arma::mat Supervised_learning<flight_controller_t,
		     simulator_t>::
insert_absolute_features(std::vector<Quadcopter::Action> actions)
{
  arma::mat features;
  arma::rowvec row;
  auto it = states_.rbegin();
  it = std::next(it, 1);
  for (int i = 0; i < 7; ++i) {
    /*  State */
    row << (*it).distances_3D().f1
	<< (*it).distances_3D().f2
	<< (*it).height_difference()
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(0)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(1)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(2)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(3)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(4)
	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(5)
      	<< mtools_.to_one_hot_encoding(action_follower_.back(), 7).at(6)
	<< states_.back().distances_3D().f1
	<< states_.back().distances_3D().f2
	<< states_.back().height_difference()
      /*  Action encoded as 1, and 0, add 7 times to represent 7 actions */
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(0)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(1)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(2)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(3)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(4)
	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(5)
    	<< mtools_.to_one_hot_encoding(actions.at(i), 7).at(6);
    /*  Create a matrix of several rows, each one is added to on the top */
    features.insert_rows(0, row);
  }
  /*  We need to transpose the matrix, since mlpack is column major */
  features = features.t();
  /*  The return features need to be used in the model in order to
      give back the best action with highest score */
  return features;
}

template <class flight_controller_t,
	  class simulator_t>
int Supervised_learning<flight_controller_t,
	       simulator_t>::
index_of_best_action_classification(arma::mat matrix)
{
  int value = 0;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    value = arma::index_max(matrix.col(0));
  }
  return value;
}

template <class flight_controller_t,
	  class simulator_t>
int Supervised_learning<flight_controller_t,
	       simulator_t>::
index_of_best_action_regression(arma::mat matrix)
{
  std::vector<double> distances = estimate_action_from_distance(matrix);
  std::reverse(distances.begin(), distances.end());
  LogInfo() << "Sum of distances: " << distances;
  int value =
    std::min_element(distances.begin(),
		     distances.end()) - distances.begin();
  return value;
}

template <class flight_controller_t,
	  class simulator_t>
std::vector<double> Supervised_learning<flight_controller_t,
					simulator_t>::
estimate_action_from_distance(arma::mat matrix)
{
  std::vector<double> sum_of_distances;
  lt::triangle<double> distances;
  double height_diff;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    /*  0 index is the height, not considered yet */
    /*  Consider only f1 and f2 */
    distances.f1 = std::fabs(original_dist_.f1 - matrix(i, 0));
    distances.f2 = std::fabs(original_dist_.f2 - matrix(i, 1));
    height_diff = std::fabs(height_diff_ - matrix(i, 2));
    sum_of_distances.push_back(distances.f1 + distances.f2 + height_diff);
  }
  return sum_of_distances;
}

template <class flight_controller_t,
	  class simulator_t>
double Supervised_learning<flight_controller_t,
			   simulator_t>::
real_time_loss(arma::mat matrix, arma::uword index_of_best_estimation)
{
  double loss = std::pow((matrix(index_of_best_estimation, 1) - states_.back().distances_3D().f1), 2) +
    std::pow((matrix(index_of_best_estimation, 2) - states_.back().distances_3D().f2), 2);
  return loss;
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t,
		simulator_t>::
generate_trajectory_using_model(bool random_leader_action,
				bool stop_down_action)
{
    mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> classification_model;

    mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
		     mlpack::ann::RandomInitialization> regression_model;
    
    if (classification_) {  
      mlpack::data::Load("model.txt", "model", classification_model, true);
    } else if(regression_) {
      mlpack::data::Load("model.txt", "model", regression_model, true);
    }
    
  /* We need to pass State, nextState, and all possible action */
  std::vector<std::thread> threads;
  
  if (random_leader_action == true) {
    action_leader_
      = robot_.random_action_generator_with_only_opposed_condition(saved_leader_action_);
    saved_leader_action_ = action_leader_;
  } else if (stop_down_action == true) {
    while (action_leader_ == Quadcopter::Action::down) {
      action_leader_ =
	robot_.random_action_generator_with_only_opposed_condition(saved_leader_action_);
    }
    saved_leader_action_ = action_leader_;      
  } else {
    action_leader_ = saved_leader_action_;
  }
    
    /* Get the next state at time t  */
    Quadcopter::State<simulator_t> state(sim_interface_);
  states_.push_back(state);
  
  action_follower_.push_back(Quadcopter::Action::NoMove);
  
  
  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("l",
								     action_leader_,
								     1000);
				}));
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f1",
								     action_leader_,
								     1000);
				}));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /* Get the next state at time t + 1  */
  Quadcopter::State<simulator_t> nextState(sim_interface_);
  states_.push_back(nextState);

  /*  We need to predict the action for the follower using h(S)*/
  /*  Extract state and push it into the model with several actions */
  /*  Take the action index for the highest class
      given back by the model */

  std::vector<Quadcopter::Action> possible_action  =
    robot_.possible_actions();

  /*  Test the trained model using the absolute gazebo distance feature */
  arma::mat features = insert_absolute_features(possible_action);
  arma::mat label;
  
  if (classification_) {
    classification_model.Predict(features, label);  
  } else if (regression_) {
    regression_model.Predict(features, label);
  }
  /* Log the controler prediction to improve accuracy*/
  arma::rowvec vec  = label.row(0);
  controller_predictions_ = mtools_.to_std_vector(vec);
  
  /* Transpose to the original format */
  features = features.t();
  label = label.t();

  LogInfo() << "True 3D distance: " << states_.back().distances_3D();
  LogInfo() << "Size of features: " << arma::size(features);
  LogInfo() << features;
  LogInfo() << label;

  int value = index_of_best_action_regression(label);
  LogInfo() << value;

  /*  Get the follower action now !! and store it directly */
  action_follower_.push_back(robot_.int_to_action(value));

  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f2",
								     action_follower_.back(),
								     1000);
  				   
  				}));
  
  for (auto& thread : threads) {
    thread.join();
  }

  /*  Get error of deformation to improve percision later and to
      verify the model accuracy */
  Quadcopter::State<simulator_t> finalState(sim_interface_);
  states_.push_back(finalState);

  double loss = real_time_loss(label, value);
  LogInfo() << "Real time loss: " << loss;
  step_errors_.push_back(mtools_.deformation_error_one_follower
			(original_dist_, nextState.distances_3D()));
  return;
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t, simulator_t>::
run(const Settings& settings)
{
  robot_.init();
  bool classification_ = settings.classification();
  bool regression_ = settings.regression();
  /*  Recover the initial state as an observer state */
  /*  This state will be used directly instead of original_dist */
  Quadcopter::State<simulator_t> ObserverState(sim_interface_);
  original_dist_ = ObserverState.distances_3D();
  height_diff_ = ObserverState.height_difference();
  
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads:
     * Arm + takeoff + offboard mode + moving + land
     * Then: repeat each episode.
     */
    LogInfo() << "Episode : " << episode_ ;

    /* Stop the episode if one of the quad has fallen to arm */   
    stop_episode_ = false;
    bool arm = swarm_.arm();
    if (!arm)
      stop_episode_ = true;

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff(25);
    if (!takeoff)
      stop_episode_ = true;
    
    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed();

    /*  Switch to offboard mode, Allow the control */
    bool offboard_mode = swarm_.start_offboard_mode();
    if (!offboard_mode)
      stop_episode_ = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));
			
  	/*  Handle fake takeoff.. */
   if (sim_interface_->positions().f1.z < 8
       or sim_interface_->positions().f2.z < 8) {
      stop_episode_ = true;      
   }
			      
    if (!stop_episode_) {

      count_ = 0;
      std::vector<lt::triangle<double>> new_triangle;

      while (count_ < 500) {
	/*  Do online learning... */
        Quadcopter::Reward reward =
	  Quadcopter::Reward::very_bad;

	if (count_ == 0 ) {
	  generate_trajectory_using_model(true, false);
	  //Change each 10 times the direction of the leader
	} else if (count_ % 10 == 0) {
	  generate_trajectory_using_model(true, false);
	} else if (sim_interface_->positions().f1.z < 15
		   or sim_interface_->positions().f2.z < 15) {
	    generate_trajectory_using_model(false, true);	  
	} else {
	  generate_trajectory_using_model(false, false);
	}

	lt::positions<lt::position3D<double>> new_positions = sim_interface_->positions();

	LogInfo() << "New positions : " << new_positions;
		
	new_triangle.push_back(mtools_.triangle_side_3D(new_positions));
	
	if (classification_) {
	  if (count_ == 0 ) {
	    reward = robot_.action_evaluator(original_dist_,
					     new_triangle.at(count_));
	    
	  } else {
	    reward = robot_.action_evaluator(new_triangle.at(count_ -1),
					     new_triangle.at(count_));
	  }
	}       	     
	/* Log online dataset */	
	if (classification_) {
	  data_set_.save_csv_data_set(states_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				      states_.back(),
				      controller_predictions_,
				      mtools_.to_one_hot_encoding(reward, 4)
				      );
	}
	
	if (regression_) {
	  auto it = states_.begin();
	  it = std::next(it, 1);
	  
	  data_set_.save_csv_data_set(states_.front(),       
				      mtools_.to_one_hot_encoding(action_follower_.front(), 7),
				      *(it),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 7),
				      states_.back()
				      );
	  
	}
	controller_predictions_.clear();
	states_.clear();
	action_follower_.clear();
	time_step_vector_.push_back(count_);
	
	/*  Need to verify that the controller is working,
	    use the triangle test to figure out after each iteration*/
	if (mtools_.is_triangle(mtools_.triangle_side_3D(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  robot_.save_controller_count(count_);
	  break;
	}
	++count_;
      }
    }
    /*  Add 1 to adjust count to one instead of starting by zero */
    count_ = count_ + 1;
    /*  Save a version of the time steps to create a histogram */
    mtools_.histogram(count_);
    data_set_.save_histogram(mtools_.get_histogram<int>());

    /*  Get the flight error as the mean of the step error */
    double mean_error = mtools_.mean(step_errors_);

    if (mean_error != -1) {
      data_set_.save_error_file(mean_error);
      flight_errors_.push_back(mean_error);
    }
   
    step_errors_.clear();
    swarm_.land();
    
    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();
    
    LogInfo() << "The quadcopters have been resetted...";    
    std::this_thread::sleep_for(std::chrono::seconds(35));
  }
}
