# include "swarm_device.hh"

template<class flight_controller_t>
SwarmDevice<flight_controller_t>::
SwarmDevice(std::vector<std::shared_ptr<flight_controller_t>> quads)
  :iris_x_(std::move(quads)),
   speed_(configs_.speed())   
{}

template <class flight_controller_t>
void SwarmDevice<flight_controller_t>::
one_quad_execute_trajectory(std::string label,
			    Quadcopter::Action action)
{
  int quad_number = 0;

  if (label == "l") {
    quad_number = 0;
  } else if (label == "f1") {
    quad_number = 1;
  } else if (label == "f2") {
    quad_number = 2;
  }

  if (action == Quadcopter::Action::left) {
    iris_x_.at(quad_number)->left(speed_);

  } else if (action == Quadcopter::Action::right) {
    iris_x_.at(quad_number)->right(speed_);

  } else if (action == Quadcopter::Action::forward) {
    iris_x_.at(quad_number)->forward(speed_);

  } else if (action == Quadcopter::Action::backward) {
    iris_x_.at(quad_number)->backward(speed_);

  } else if (action == Quadcopter::Action::up) {
    iris_x_.at(quad_number)->up(speed_);

  } else if (action == Quadcopter::Action::down) {
    iris_x_.at(quad_number)->down(speed_);
  }
}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
arm()
{
  bool arm;
  for (auto it : iris_x_){
    arm = it->arm();
    if (!arm)
      return false;
  }
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
takeoff(float meters)
{
  bool takeoff;
  for (auto it : iris_x_) {
    takeoff = it->takeoff(meters);
    if (!takeoff)
      return false;
  }
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
