/**
https://sdk.dronecode.org/en/examples/transition_vtol_fixed_wing.html  * @file controller.hh
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-06-13
 */

# include <chrono>
# include <cmath>
# include <dronecode_sdk/action.h>
# include <dronecode_sdk/telemetry.h>
# include <dronecode_sdk/dronecode_sdk.h>
# include <dronecode_sdk/offboard.h>
# include <iostream>
# include <thread>
# include <memory>

# include "settings.hh"


using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;


#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour


class Controller 
{

  
public:

  Controller(Settings settings);

  ActionResult arm();
  
  bool takeoff();
  bool land();
  void goUp();
  void goDown();
  void goRight();
  void goLeft();
  void forward();
  void backward();
  void turnToLeft();
  void turnToRight();


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



  
