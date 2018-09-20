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
  
  Controller();
  
  bool takeoff(std::shared_ptr<dronecode_sdk::Action> action);
  bool land(std::shared_ptr<dronecode_sdk::Action> action);
  void goUp(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void goDown(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void goRight(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void goLeft(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void forward(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void backward(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void turnToLeft(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  void turnToRight(std::shared_ptr<dronecode_sdk::Offboard> offboard);


  void init_speed(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  Offboard::Result start_offboard_mode(std::shared_ptr<dronecode_sdk::Offboard> offboard);
  
  bool discover_system(DronecodeSDK& dc);
  ConnectionResult connect_to_quad(DronecodeSDK& dc,
				   std::string connection_url);
  ActionResult arm(std::shared_ptr<dronecode_sdk::Action> action);
  
  void get_position(std::shared_ptr<dronecode_sdk::Telemetry> telemetry);
  Telemetry::PositionVelocityNED get_position_ned(std::shared_ptr<dronecode_sdk::Telemetry> telemetry);
  
  void quad_health(std::shared_ptr<dronecode_sdk::Telemetry> telemetry);
  Telemetry::Result set_rate_result(std::shared_ptr<dronecode_sdk::Telemetry> telemetry);
  
private:
  
  DronecodeSDK dc_;
  
  std::shared_ptr<dronecode_sdk::Telemetry> telemetry_;
  std::shared_ptr<dronecode_sdk::Action> action_;
  std::shared_ptr<dronecode_sdk::Offboard> offboard_;



  
};



  
