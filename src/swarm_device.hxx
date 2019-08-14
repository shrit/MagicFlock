# include "swarm_device.hh"

template<class flight_controller_t>
SwarmDevice<flight_controller_t>::
SwarmDevice(std::vector<std::shared_ptr<flight_controller_t>> quads)
  :iris_x_(std::move(quads))
{}

template<class flight_controller_t>
bool SwarmDevice<flight_controller_t>::
takeoff(float meters)
{
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
