/**
 * @file controller.hh
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-06-13
 */

# include <chrono>
# include <cmath>
# include <dronecore/action.h>
# include <dronecore/telemetry.h>
# include <dronecore/dronecore.h>
# include <dronecore/offboard.h>
# include <iostream>
# include <thread>
# include <memory>

using namespace dronecore;
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
  
  bool takeoff(std::shared_ptr<dronecore::Action> action);
  bool land(std::shared_ptr<dronecore::Action> action);
  bool goUp(std::shared_ptr<dronecore::Offboard> offboard);
  bool goDown(std::shared_ptr<dronecore::Offboard> offboard);
  bool goRight(std::shared_ptr<dronecore::Offboard> offboard);
  bool goLeft(std::shared_ptr<dronecore::Offboard> offboard);
  bool forward(std::shared_ptr<dronecore::Offboard> offboard);
  bool backward(std::shared_ptr<dronecore::Offboard> offboard);
  bool turnToLeft(std::shared_ptr<dronecore::Offboard> offboard);
  bool turnToRight(std::shared_ptr<dronecore::Offboard> offboard);
  
  bool discover_system(DroneCore& dc);
  ConnectionResult connect_to_quad(DroneCore& dc,
				   std::string connection_url);
  ActionResult arm(std::shared_ptr<dronecore::Action> action);
  
  void get_position(std::shared_ptr<dronecore::Telemetry> telemetry);
  void quad_health(std::shared_ptr<dronecore::Telemetry> telemetry);
  Telemetry::Result set_rate_result(std::shared_ptr<dronecore::Telemetry> telemetry);
  
private:
  
  DroneCore dc_;
  
  std::shared_ptr<dronecore::Telemetry> telemetry_;
  std::shared_ptr<dronecore::Action> action_;
  std::shared_ptr<dronecore::Offboard> offboard_;
  
  
};



  
