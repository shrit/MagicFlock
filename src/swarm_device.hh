#pragma once

/*  Standard C++ includes */
# include <algorithm>
# include <chrono>
# include <thread>
# include <vector>

/*  Local includes */
# include "config_ini.hh"
# include "global.hh"

template<class flight_controller_t>
class SwarmDevice {

public:
  
  SwarmDevice(std::vector<std::shared_ptr<flight_controller_t>> quads);

  void one_quad_execute_trajectory(std::string label,
				   Quadcopter::Action action,
				   unsigned int milliseconds);
  bool arm();
  void init_speed();
  bool start_offboard_mode();
  bool land();
  lt::positions<lt::position_GPS<double>> positions_GPS();
  bool takeoff(float meters);  
  
  SwarmDevice(SwarmDevice const&) = delete;
  SwarmDevice(SwarmDevice &&) = default;
  
private:
  Configs configs_;
  std::vector<std::shared_ptr<flight_controller_t>> iris_x_;
  float speed_;
};

# include "swarm_device.hxx"
