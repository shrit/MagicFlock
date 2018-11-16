/**
 * @file px4_device.hh
 * @brief Device code, that allow user to handle the quadcopters
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-06-13
 */

/*  C++ Standard includes */

# include <chrono>
# include <cmath>
# include <iostream>
# include <thread>
# include <memory>

/*  DronecodeSDK includes */

# include <dronecode_sdk/action.h>
# include <dronecode_sdk/telemetry.h>
# include <dronecode_sdk/dronecode_sdk.h>
# include <dronecode_sdk/offboard.h>

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

class Px4Device // : std::enable_shared_from_this<Px4Device>
{
  
public:

  Px4Device(lt::connection_type socket, lt::port_type port);

  ActionResult arm();
  
  bool takeoff();
  bool land();
  void goUp(float speed);
  void goDown(float speed);
  void goRight(float speed);
  void goLeft(float speed);
  void forward(float speed);
  void backward(float speed);
  void turnToLeft(float speed);
  void turnToRight(float speed);


  void init_speed();
  Offboard::Result start_offboard_mode();
  
  bool discover_system();
  ConnectionResult connect_to_quad(std::string connection_url);

  
  void print_position();
  void async_position_ned();
  Telemetry::PositionVelocityNED get_position_ned();
  
  void quad_health();
  Telemetry::Result set_rate_result();
  
private:
  
  DronecodeSDK dc_;
  Telemetry::PositionVelocityNED position_ned_ ;
  std::shared_ptr<dronecode_sdk::Telemetry> telemetry_;
  std::shared_ptr<dronecode_sdk::Action> action_;
  std::shared_ptr<dronecode_sdk::Offboard> offboard_;  
  
};

  
