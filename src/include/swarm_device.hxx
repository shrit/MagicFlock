#pragma once

# include "swarm_device.hh"

template<class flight_controller_t>
SwarmDevice<flight_controller_t>::
SwarmDevice(std::vector<std::shared_ptr<flight_controller_t>> quads)
  :iris_x_(std::move(quads))
{}

template <class flight_controller_t>
void SwarmDevice<flight_controller_t>::
one_quad_execute_trajectory(unsigned int id,
			    Actions::Action action,
			    int speed,
			    unsigned int milliseconds)
{
  if (action == Actions::Action::left) {
    iris_x_.at(id)->left(speed, milliseconds);

  } else if (action == Actions::Action::right) {
    iris_x_.at(id)->right(speed, milliseconds);

  } else if (action == Actions::Action::forward) {
    iris_x_.at(id)->forward(speed, milliseconds);

  } else if (action == Actions::Action::backward) {
    iris_x_.at(id)->backward(speed, milliseconds);

  } else if (action == Actions::Action::up) {
    iris_x_.at(id)->up(speed, milliseconds);

  } else if (action == Actions::Action::down) {
    iris_x_.at(id)->down(speed, milliseconds);
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
arm_specific_quadrotor(unsigned int id)
{
  bool arm = iris_x_.at(id)->arm();
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
init_speed_specific_quadrotor(unsigned int id)
{
  iris_x_.at(id)->init_speed();  
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
start_offboard_mode_specific_quadrotor(unsigned int id)
{
  bool offboard_mode = iris_x_.at(id)->start_offboard_mode();
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
takeoff_specific_quadrotor(float meters, unsigned int id)
{
  bool takeoff = iris_x_.at(id)->takeoff(meters);  
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
land_specific_quadrotor(unsigned int id)
{
  bool land = iris_x_.at(id)->land();  
  if (!land)
    return false;

  return true;    
}

template<class flight_controller_t>
std::vector<lt::position_GPS<double>> SwarmDevice<flight_controller_t>::
positions_GPS()
{
  std::vector<lt::position_GPS<double>> positions;
  positions.push_back(iris_x_.at(0)->get_position_GPS());
  positions.push_back(iris_x_.at(1)->get_position_GPS());
  positions.push_back(iris_x_.at(2)->get_position_GPS());
  return positions;
}
