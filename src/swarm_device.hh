#pragma once

/*  Standard C++ includes */
# include <algorithm>
# include <chrono>
# include <thread>
# include <vector>


template<class flight_controller_t>
class SwarmDevice {

public:
  
  SwarmDevice(std::vector<std::shared_ptr<flight_controller_t>> quads);

  bool takeoff(float meters);
  bool land();
  
  SwarmDevice(SwarmDevice const&) = delete;
  
  SwarmDevice(SwarmDevice &&) = default;
  
private:
  std::vector<std::shared_ptr<flight_controller_t>> iris_x_;
};

# include "swarm_device.hxx"
