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
				   Actions::Action action,
				   unsigned int milliseconds);
  bool arm();
  bool arm_specific_quadrotor(std::string quadrotor_name);
  
  void init_speed();
  void init_speed_specific_quadrotor(std::string quadrotor_name);
    
  bool start_offboard_mode();
  bool start_offboard_mode_specific_quadrotor(std::string quadrotor_name);
    
  bool land();
  bool land_specific_quadrotor(std::string quadrotor_name);
  
  lt::positions<lt::position_GPS<double>> positions_GPS();
  int quadrotor_label_2_number(std::string label);

  bool takeoff(float meters);  
  bool takeoff_specific_quadrotor(float meters, std::string quadrotor_name);
  
  SwarmDevice(SwarmDevice const&) = delete;
  SwarmDevice(SwarmDevice &&) = default;
  
private:
  Configs configs_;
  std::vector<std::shared_ptr<flight_controller_t>> iris_x_;
  float speed_;
};

# include "swarm_device.hxx"
