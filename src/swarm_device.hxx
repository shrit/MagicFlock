#pragma once

# include "swarm_device.hh"

template<class flight_controller_t>
SwarmDevice<flight_controller_t>::
SwarmDevice(std::vector<std::shared_ptr<flight_controller_t>> quads)
  :iris_x_(std::move(quads)),
   speed_(configs_.speed())   
{}

template <class flight_controller_t>
int SwarmDevice<flight_controller_t>::
quadrotor_label_2_number(std::string label)
{
  int quad_number = 0;
  
  if (label == "l") {
    quad_number = 0;
  } else if (label == "f1") {
    quad_number = 1;
  } else if (label == "f2") {
    quad_number = 2;
  }
  return quad_number;
}

template <class flight_controller_t>
void SwarmDevice<flight_controller_t>::
one_quad_execute_trajectory(std::string label,
			    Quadcopter::Action action,
			    unsigned int milliseconds)
{
  int quad_number = 0;
  quad_number = quadrotor_label_2_number(label);

  if (action == Quadcopter::Action::left) {
    iris_x_.at(quad_number)->left(speed_, milliseconds);

  } else if (action == Quadcopter::Action::right) {
    iris_x_.at(quad_number)->right(speed_, milliseconds);

  } else if (action == Quadcopter::Action::forward) {
    iris_x_.at(quad_number)->forward(speed_, milliseconds);

  } else if (action == Quadcopter::Action::backward) {
    iris_x_.at(quad_number)->backward(speed_, milliseconds);

  } else if (action == Quadcopter::Action::up) {
    iris_x_.at(quad_number)->up(speed_, milliseconds);

  } else if (action == Quadcopter::Action::down) {
    iris_x_.at(quad_number)->down(speed_, milliseconds);
  }
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
arm()
{
  bool arm = true;
  std::vector<std::thread> threads;
  
  threads.push_back(std::thread([&](){
				  arm = iris_x_.at(0)->arm();		    
				})); 
  
  threads.push_back(std::thread([&](){			
				  arm = iris_x_.at(1)->arm();		    
				}));			
  
  threads.push_back(std::thread([&](){			
				  arm = iris_x_.at(2)->arm();		    
				}));
  
  for (auto& thread : threads) {
    thread.join();
  }
  
  if (!arm)
    return false;
  
  return true;	
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
arm_specific_quadrotor(std::string quadrotor_name)
{
  int quad_number = quadrotor_label_2_number(quadrotor_name);
  bool arm = iris_x_.at(quad_number)->arm();
  if (!arm)
    return false;
  
  return true;	
}

template<class flight_controller_t>
void SwarmDevice<flight_controller_t>::
init_speed()
{
  for (auto it : iris_x_) {
    it->init_speed();
  }
}

template<class flight_controller_t>
void SwarmDevice<flight_controller_t>::
init_speed_specific_quadrotor(std::string quadrotor_name)
{
  int quad_number = quadrotor_label_2_number(quadrotor_name);
  iris_x_.at(quad_number)->init_speed();  
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
start_offboard_mode()
{
  bool offboard_mode;
  for (auto it : iris_x_) {
    offboard_mode = it->start_offboard_mode();
    if (!offboard_mode)
      return false;
  }
  return true;
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
start_offboard_mode_specific_quadrotor(std::string quadrotor_name)
{
  int quad_number = quadrotor_label_2_number(quadrotor_name);
  bool offboard_mode = iris_x_.at(quad_number)->start_offboard_mode();
  if (!offboard_mode)
    return false;
  
  return true;
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
takeoff(float meters)
{
  bool takeoff = true;
  std::vector<std::thread> threads;

  threads.push_back(std::thread([&](){
				  takeoff = iris_x_.at(0)->takeoff(meters);		    
				})); 
  							
  threads.push_back(std::thread([&](){			
				  takeoff = iris_x_.at(1)->takeoff(meters);		    
				}));			
  							
  threads.push_back(std::thread([&](){			
				  takeoff = iris_x_.at(2)->takeoff(meters);		    
				}));
  
  for (auto& thread : threads) {
    thread.join();
  }
  if (!takeoff)
      return false;

  return true;
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
takeoff_specific_quadrotor(float meters, std::string quadrotor_name)
{
  int quad_number = quadrotor_label_2_number(quadrotor_name);
  bool takeoff = iris_x_.at(quad_number)->takeoff(meters);
  
  if (!takeoff)
    return false;
  
  return true;
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
land()
{
  bool land = true;
  std::vector<std::thread> threads;
  
  threads.push_back(std::thread([&](){
				  land = iris_x_.at(0)->land();		    
				}));
  
  threads.push_back(std::thread([&](){
				  land = iris_x_.at(1)->land();		    
				}));
  
  threads.push_back(std::thread([&](){
				  land = iris_x_.at(2)->land();		    
				}));
    
  for (auto& thread : threads) {
    thread.join();
  }
  if (!land)
    return false;

  return true;    
}
template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
land_specific_quadrotor(std::string quadrotor_name)
{
  int quad_number = quadrotor_label_2_number(quadrotor_name);
  bool land = iris_x_.at(quad_number)->land();
  
  if (!land)
    return false;

  return true;    
}

template<class flight_controller_t>
lt::positions<lt::position_GPS<double>> SwarmDevice<flight_controller_t>::
positions_GPS()
{
  lt::positions<lt::position_GPS<double>> positions;
  positions.leader = iris_x_.at(0)->get_position_GPS();
  positions.f1 = iris_x_.at(1)->get_position_GPS();
  positions.f2 = iris_x_.at(2)->get_position_GPS();
  return positions;
}
