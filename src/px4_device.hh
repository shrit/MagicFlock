# ifndef PX4_DEVICE_HH_
# define PX4_DEVICE_HH_

/**
 * @file px4_device.hh
 * @brief Device code, that allow user to handle the quadcopters
 * using keyboard or joystick
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-06-13
 */

/*  C++ Standard includes */

# include <chrono>
# include <cmath>
# include <iostream>
# include <thread>
# include <memory>
# include <mutex>
# include <atomic>
# include <future>

/*  DronecodeSDK includes */

# include <dronecode_sdk/action.h>
# include <dronecode_sdk/calibration.h>
# include <dronecode_sdk/dronecode_sdk.h>
# include <dronecode_sdk/offboard.h>
# include <dronecode_sdk/telemetry.h>

/*  local includes */

# include "global.hh"

using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;


#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

namespace lt = local_types;

/*  TODO list: 
    /*
    * Add error management 
    * Put the code inside the examples of the api
    */


class Px4Device 
{
  
public:

  Px4Device(lt::connection_type socket, lt::port_type port);

  Action::Result arm();
  Action::Result reboot();
  
  bool takeoff();
  bool land();
  
  bool return_to_launch();
  bool set_altitude_rtl_max(float meter);
  
  void up(float speed);
  void down(float speed);
  void right(float speed);
  void left(float speed);
  void forward(float speed);
  void backward(float speed);
  void turnToLeft(float speed);
  void turnToRight(float speed);

  void init_speed();
  Offboard::Result start_offboard_mode();
  
  bool discover_system();
  ConnectionResult connect_to_quad(std::string connection_url);

  void print_position();
  void position_ned();

  Calibration::calibration_callback_t
  create_calibration_callback(std::promise<void> &calibration_promise);

  void calibrate_accelerometer();
  Telemetry::PositionVelocityNED position() const;
    
  void quad_health();
  Telemetry::Result set_rate_result();
  
  /*  Handles plugin results. */
  
  inline void action_error_exit(Action::Result result, const std::string &message);
  inline void offboard_error_exit(Offboard::Result result, const std::string &message);
  inline void connection_error_exit(ConnectionResult result, const std::string &message);
  
private:
  
  DronecodeSDK dc_;
  
  //  mutable std::mutex _position_ned_mutex{};
  Telemetry::PositionVelocityNED _position_ned{{0, 0, 0}, {0, 0, 0}};
  std::shared_ptr<dronecode_sdk::Telemetry> telemetry_;
  std::shared_ptr<dronecode_sdk::Action> action_;
  std::shared_ptr<dronecode_sdk::Offboard> offboard_;
  std::shared_ptr<dronecode_sdk::Calibration> calibration_;  
  
};



#endif
