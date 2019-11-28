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

  void one_quad_execute_trajectory(unsigned int id,
				   Actions::Action action,
				   unsigned int milliseconds);
  bool arm();
  bool arm_specific_quadrotor(unsigned int id);
  
  void init_speed();
  void init_speed_specific_quadrotor(unsigned int id);
    
  bool start_offboard_mode();
  bool start_offboard_mode_specific_quadrotor(unsigned int id);
    
  bool land();
  bool land_specific_quadrotor(unsigned int id);
  
  lt::positions<lt::position_GPS<double>> positions_GPS();

  bool takeoff(float meters);  
  bool takeoff_specific_quadrotor(float meters, unsigned int id);
  
  SwarmDevice(SwarmDevice const&) = delete;
  SwarmDevice(SwarmDevice &&) = default;
  
private:
  Configs configs_;
  std::vector<std::shared_ptr<flight_controller_t>> iris_x_;
  float speed_;
};

# include "swarm_device.hxx"