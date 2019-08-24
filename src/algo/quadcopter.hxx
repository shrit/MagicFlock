#pragma once

# include "quadcopter.hh"

Quadcopter::Quadcopter()
  :distribution_(0.0, 1.0),
   distribution_int_(0, 5),
   generator_(random_dev())
{}

void Quadcopter::init()
{
  data_set_.init_dataset_directory();
}

template <class simulator_t>
Quadcopter::State<simulator_t>::State(std::shared_ptr<simulator_t> sim_interface):
  sim_interface_(std::move(sim_interface)),
  pmodel_(sim_interface_)
{
  rssi_ = sim_interface_->rssi();
  height_ = sim_interface_->positions().f2.z;
  z_orinetation_  = sim_interface_->orientations().f2.z;
  dists_2D_ =   mtools_.triangle_side_2D(sim_interface_->positions());
  dists_3D_ =   mtools_.triangle_side_3D(sim_interface_->positions());
  e_dists_ = pmodel_.distances_2D();
}

template <class simulator_t>
lt::rssi<double> Quadcopter::State<simulator_t>::
signal_strength () const
{
  return rssi_ ;
}

template <class simulator_t>
double Quadcopter::State<simulator_t>::
height () const
{/*  need to make it according to what quad?? */
  return height_;
}

template <class simulator_t>
double Quadcopter::State<simulator_t>::
orientation () const
{
  return z_orinetation_;
}

template <class simulator_t>
lt::triangle<double> Quadcopter::State<simulator_t>::
distances_2D () const
{
  return dists_2D_;
}

template <class simulator_t>
lt::triangle<double> Quadcopter::State<simulator_t>::
distances_3D () const
{
  return dists_3D_;
}

template <class simulator_t>
lt::triangle<double> Quadcopter::State<simulator_t>::
estimated_distances () const
{
  return e_dists_;
}

Quadcopter::Reward Quadcopter::
action_evaluator(const lt::triangle<double>& old_dist,
		 const lt::triangle<double>& new_dist)
{
  LogInfo() << "F1 differences: " << std::fabs(old_dist.f1 - new_dist.f1);
  LogInfo() << "F2 differences: " << std::fabs(old_dist.f2 - new_dist.f2);

  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);

   Reward reward = Reward::Unknown;

  if (0.5  > diff_f1 + diff_f2 ) {
    reward = Reward::very_good;
  } else if ( 1.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 0.5 ) {
    reward = Reward::good;
  } else if ( 1.5  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.0 ) {
    reward = Reward::bad;
  } else if ( 2.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.5 ) {
    reward = Reward::very_bad;
  }
  return reward;
}

/* Get the best action from the model according to the best values */
Quadcopter::Action Quadcopter::
action_follower(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  Quadcopter::Action action = Quadcopter::Action::NoMove;
  /*  Access matrix values according to a given index  */
  /*  Only one action exist that equal 1 in each row of 
   a matrix */
  
  if (features(index, 5) == 1) {
    action =  Quadcopter::Action::forward;
  } else if (features(index, 6) == 1) {
    action =  Quadcopter::Action::backward;
  } else if (features(index, 7) == 1) {
    action =  Quadcopter::Action::left;
  } else if (features(index, 8) == 1) {
    action =  Quadcopter::Action::right;
  } else if (features(index, 9) == 1) {
    action =  Quadcopter::Action::up;
  } else if (features(index, 10) == 1) {
    action =  Quadcopter::Action::down;
  }
  return action;
}

double Quadcopter::true_score(const lt::triangle<double>& old_dist,
			      const lt::triangle<double>& new_dist)
{
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);

  return diff_f1 + diff_f2;
}
/*  NOT tested yet, Do not use, it requires some verifications */
int Quadcopter::evaluation_score(const lt::triangle<double>& old_dist,
		     const lt::triangle<double>& new_dist,
		     const double altitude,
		     const double old_altitude)
{  
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);
  double diff_f3 = std::fabs(old_dist.f3 - new_dist.f3);
  double diff_altitude = std::fabs(altitude - old_altitude);
  
  int score = -1;

  /*  Use comparsion and increase the score each time we execute the
      good action. If the executed action is sitl the same and the
      score start decreasing we can increase the speed or change the
      action*/
  
  if (0.2 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 0.2) {
    score = 10;
  } else if (0.4 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 0.4) {
    score = 9;
  } else if (0.6 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 0.6) {
    score = 8;
  } else if (0.8 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 0.8) {
    score = 7;
  } else if (1 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 1) {
    score = 6;
  } else if (1.2 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 1.2) {
    score = 5;
  } else if (1.4 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 1.4) {
    score = 4;
  } else if (1.6 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 1.6) {
    score = 3;
  } else if (1.8 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 1.8) {
    score = 2;
  } else if (2.0 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 2.0) {
    score = 1;
  } else if (2.2 > diff_f1 + diff_f2 + diff_f3 and diff_altitude < 2.2) {
    score = 0; 
  }
  return score;
}

void Quadcopter::
save_controller_count(double value)
{
  data_set_.save_count_file(value);
}

std::vector<Quadcopter::Action> Quadcopter::
possible_actions() const
{ return possible_actions_; }

Quadcopter::Action Quadcopter::
random_action_generator()
{
  int random_action = distribution_int_(generator_);

  LogInfo() << "Random action value: " << random_action ;

  Action action = Action::forward ;

  if (random_action == 0) {
    action = Action::forward;
  } else if (random_action == 1) {
    action = Action::backward ;
  } else if (random_action == 2) {
    action = Action::left;
  } else if (random_action == 3) {
    action = Action::right;
  } else if (random_action == 4) {
    action = Action::up;
  } else if (random_action == 5) {
    action = Action::down;
  }
      
   return action;
}

template <class simulator_t>
inline std::ostream& operator<< (std::ostream& out, const Quadcopter::State<simulator_t>& s)
{
  out << s.height() << ","
      << s.distances_3D() << ","
      << s.orientation() ;
  return out;
}
