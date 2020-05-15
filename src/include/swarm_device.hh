#pragma once

/*  Standard C++ includes */
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

/*  Local includes */
#include "check_swarm_shape.hh"
#include "continuous_actions.hh"
#include "discret_actions.hh"
#include "logger.hh"
#include "position_gps.hh"

template<class QuadrotorType>
class SwarmDevice
{

public:
  SwarmDevice(std::vector<std::shared_ptr<QuadrotorType>> quads);

  void one_quad_execute_trajectory(unsigned int id,
                                   DiscretActions::Action action,
                                   int speed,
                                   unsigned int milliseconds);

  void one_quad_execute_trajectory(unsigned int id, ContinuousActions action);

  bool arm();
  void arm_async();
  bool arm_specific_quadrotor(unsigned int id);

  void init_speed();
  void init_speed_specific_quadrotor(unsigned int id);

  bool start_offboard_mode();
  void start_offboard_mode_async();
  bool start_offboard_mode_specific_quadrotor(unsigned int id);

  void flight_mode_async();
  void landed_state_async();

  bool land();
  void land_async();
  bool land_specific_quadrotor(unsigned int id);

  std::vector<position_GPS<double>> positions_GPS();

  bool takeoff(float meters);
  void takeoff_async(float meters);
  bool takeoff_specific_quadrotor(float meters, unsigned int id);
  bool in_air(float meters);
  void in_air_async(float meters);

  bool examin_swarm_shape();
  void reset_models();

  SwarmDevice(SwarmDevice const&) = delete;
  SwarmDevice(SwarmDevice&&) = default;

private:
  std::vector<std::shared_ptr<QuadrotorType>> quads_;
  CheckShape<QuadrotorType> shape_;
};

#include "swarm_device.hxx"
