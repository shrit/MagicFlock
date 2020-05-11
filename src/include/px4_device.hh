#pragma once

/**
 * @file px4_device.hh
 * @brief An interface to MAVSDK, it allows users to handle quadcopters
 * @author: Omar Shrit <shrit@lri.fr>
 */
/*  C++ Standard includes */
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

/*  MAVSDK includes */
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/calibration/calibration.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/shell/shell.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include "logger.hh"
#include "position_gps.hh"

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using namespace ILMR;

#define ERROR_CONSOLE_TEXT "\033[31m"     // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"     // Restore normal console colour

class Px4Device : public std::enable_shared_from_this<Px4Device>
{
public:
  Px4Device(std::string socket, std::string port);

  bool arm();
  bool reboot();

  bool takeoff();
  bool takeoff(float meters);

  bool land();

  bool return_to_launch();
  bool set_altitude_rtl_max(float meters);
  bool set_takeoff_altitude(float meters);

  template<typename VecType>
  void set_velocity_vector(VecType vec)
  {
    offboard_->set_velocity_body({ vec.X(), vec.Y(), vec.Z(), 0.0f });
  }

  /*  Linear trajectories for discret actions*/
  void up(float speed);
  void down(float speed);
  void right(float speed);
  void left(float speed);
  void forward(float speed);
  void backward(float speed);

  /*  Linear trajectories with a specific duaration for discret actions*/
  void up(float speed, unsigned int milliseconds_);
  void down(float speed, unsigned int milliseconds_);
  void right(float speed, unsigned int milliseconds_);
  void left(float speed, unsigned int milliseconds_);
  void forward(float speed, unsigned int milliseconds_);
  void backward(float speed, unsigned int milliseconds_);

  /*  Turn around itself */
  void turnToLeft();
  void turnToRight();

  /*  Add circular trajectory for discret actions*/
  void forward_left(float speed);
  void forward_right(float speed);
  void backward_left(float speed);
  void backward_right(float speed);

  void init_speed();
  bool start_offboard_mode();
  bool stop_offboard_mode();

  bool discover_system();
  ConnectionResult connect_to_quad(std::string connection_url);

  /*  Position from PX4 autopilot, intilized usually by GPS */
  position_GPS<double> get_position_GPS();
  void position_async();

  /*  Position_ned */
  Telemetry::PositionVelocityNed get_position_ned() const;
  void position_ned_async();

  std::function<void(Calibration::Result, Calibration::ProgressData)>
  create_calibration_callback(std::promise<void>& calibration_promise);

  void calibrate_accelerometer();

  double DistanceFrom(std::shared_ptr<Px4Device> a);

  void quad_health();
  Telemetry::Result set_rate_result();

  void landed_state_async();
  Telemetry::LandedState landed_state() const;

  void flight_mode_async();
  Telemetry::FlightMode flight_mode() const;

  bool execute_px4_shell_command(std::string command);
  bool receive_px4_shell_reponse();

  /*  Handles plugin results. */

  inline void action_error_exit(Action::Result result,
                                const std::string& message);
  inline void offboard_error_exit(Offboard::Result result,
                                  const std::string& message);
  inline void connection_error_exit(ConnectionResult result,
                                    const std::string& message);

  Px4Device(Px4Device const&) = delete;
  Px4Device(Px4Device&&) = default;

private:
  double CalculateDistance(Telemetry::PositionVelocityNed& a,
                           Telemetry::PositionVelocityNed& b);

  Mavsdk mavsdk_;
  Telemetry::Health health_;
  Telemetry::PositionVelocityNed _position_ned{ { 0, 0, 0 }, { 0, 0, 0 } };
  Telemetry::Position position_{ 0, 0, 0, 0 };

  Telemetry::LandedState _landed_state;
  mutable std::mutex _landed_state_mutex{};

  Telemetry::FlightMode _flight_mode;
  mutable std::mutex _flight_mode_mutex{};

  Shell::ReceiveCallback _shell_reponse;

  std::shared_ptr<mavsdk::Action> action_;
  std::shared_ptr<mavsdk::Calibration> calibration_;
  std::shared_ptr<mavsdk::Offboard> offboard_;
  std::shared_ptr<mavsdk::Shell> shell_;
  std::shared_ptr<mavsdk::Telemetry> telemetry_;
};
