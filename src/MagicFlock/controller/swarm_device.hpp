#pragma once

/*  Standard C++ includes */
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

/*  Local includes */
#include <MagicFlock/actions/continuous_actions.hpp>
#include <MagicFlock/actions/discret_actions.hpp>
#include <MagicFlock/check/check_destination.hpp>
#include <MagicFlock/check/check_swarm_shape.hpp>
#include <MagicFlock/util/logger.hpp>
#include <MagicFlock/util/position_gps.hpp>

using namespace ILMR;

template<class QuadrotorType>
class SwarmDevice
{

public:
  SwarmDevice(std::vector<QuadrotorType>& quads);

  void one_quad_execute_trajectory(unsigned int id,
                                   typename QuadrotorType::Action action);

  bool arm();
  void arm_async();
  void disarm_async();
  bool arm_specific_quadrotor(unsigned int id);

  void init_speed();
  void init_speed_specific_quadrotor(unsigned int id);

  bool start_offboard_mode();
  void start_offboard_mode_async();
  bool start_offboard_mode_specific_quadrotor(unsigned int id);

  void stop_offboard_mode_async();

  void flight_mode_async();
  void landed_state_async();

  bool land();
  void land_async();
  bool land_specific_quadrotor(unsigned int id);
  void return_to_launch();
  void return_to_launch_async();
  std::vector<position_GPS<double>> positions_GPS();

  bool takeoff(float meters);
  void takeoff_async(float meters);
  bool takeoff_specific_quadrotor(float meters, unsigned int id);
  bool in_air(float meters);
  void in_air_async(float meters);

  bool examin_swarm_shape();
  bool examin_swarm_shape(double lower_threshold, double upper_threshold);
  bool examin_destination(const ignition::math::Vector3d& destination);

  void reset_models();

  template<class status>
  bool checking_status(std::vector<status> results);

  template<class status>
  bool check_landed_state(std::vector<status> results);

  SwarmDevice(SwarmDevice const&) = delete;
  SwarmDevice(SwarmDevice&&) = default;

private:
  std::vector<QuadrotorType>& quads_;
  CheckShape<QuadrotorType> shape_;
  CheckDestination<QuadrotorType> dest_;
  std::vector<typename QuadrotorType::inner_flight_controller::LandedState>
    landed_state_results_;
  std::vector<typename QuadrotorType::inner_flight_controller::OffboardResult>
    offboard_mode_results_;
  std::vector<typename QuadrotorType::inner_flight_controller::ActionResult>
    takeoff_results_;
  std::vector<typename QuadrotorType::inner_flight_controller::ActionResult>
    landed_results_;
  std::vector<typename QuadrotorType::inner_flight_controller::ActionResult>
    arming_results_;
};

#include "swarm_device_impl.hpp"
