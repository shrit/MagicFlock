OB/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-06-13
 */

extern "C" {

#include <ncurses.h>

}

#include <chrono>
#include <cmath>
#include <dronecore/action.h>
#include <dronecore/telemetry.h>
#include <dronecore/dronecore.h>
#include <dronecore/offboard.h>
#include <iostream>
#include <thread>


using namespace dronecore;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;


#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

void usage (std::string binary_name)
{
  std::cout<< NORMAL_CONSOLE_TEXT << "Usage : " << binary_name <<
    "<connection_url> " << std::endl;
}



inline void action_error_exit(ActionResult result, const std::string &message)
{
    if (result != ActionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << action_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string &message)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string &message)
{
    if (result != ConnectionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}


int takeoff (std::shared_ptr<dronecore::Action> action)
{
  std::cout << "taking off" << std::endl;
  const ActionResult takeoff_result = action->takeoff();
  if(takeoff_result != ActionResult::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      << "take off failed: "
	      << action_result_str(takeoff_result)
	      << std::endl;
    return 1;
  }
  return 0;
}


int land(std::shared_ptr<dronecore::Action> action)
{
  std::cout << "Landing..." << std::endl;
  const ActionResult land_result = action->land();
  if (land_result != ActionResult::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "Land failed:"
	      << action_result_str(land_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return 1;
  }
  return 0;
}


bool move_x(std::shared_ptr<dronecore::Offboard> offboard)
{

  offboard->set_velocity_body({2.0f, 0.0f, 0.0f, 0.0f});
  sleep_for(seconds(2));
  return true;

}

bool move_y(std::shared_ptr<dronecore::Offboard> offboard)
{

  offboard->set_velocity_body({0.0f, 2.0f, 0.0f, 0.0f});
  sleep_for(seconds(5));
  return true;
}


bool move_z(std::shared_ptr<dronecore::Offboard> offboard)
{

  offboard->set_velocity_body({0.0f, 0.0f, -3.0f, 0.0f});
  sleep_for(seconds(2));
    return true;
}

int turn_yaw(std::shared_ptr<dronecore::Offboard> offboard)
{
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 10.0f});
  sleep_for(seconds(1));
    return true;
}



int main(int argc, char** argv)
{

  DroneCore dc;

  std::string connection_url;
  connection_url = "udp://:14540";

  ConnectionResult connection_result;
  bool discovered_system = false;

  connection_result = dc.add_any_connection(connection_url);

  if (connection_result != ConnectionResult::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "Connection failed: "
	      << connection_result_str(connection_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return 1;
  }



  std::cout << "Waiting to discover system..." << std::endl;
  dc.register_on_discover([&discovered_system](uint64_t uuid) {
			    std::cout << "Discovered system with UUID: "
				      << uuid << std::endl;
			    discovered_system = true;
			  });

  sleep_for(seconds(2));


  if (!discovered_system) {
    std::cout << ERROR_CONSOLE_TEXT
	      <<"No system found, exiting." << NORMAL_CONSOLE_TEXT
	      << std::endl;
    return 1;
    }

  System& system = dc.system();

  auto telemetry = std::make_shared<Telemetry>(system);
  auto action = std::make_shared<Action>(system);
  auto offboard = std::make_shared<Offboard>(system);

  const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);

  if (set_rate_result != Telemetry::Result::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      <<"Set rate failed:"
	      << Telemetry::result_str(set_rate_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return 1;
  }



  //Quad position
  telemetry->position_async([](Telemetry::Position position){
			      std::cout << TELEMETRY_CONSOLE_TEXT
 					<< "Altitude : "
					<< position.relative_altitude_m
					<< " m"
					<< "Latitude"
					<< position.latitude_deg << " deg"
					<< "Longtitude"
					<< position.longitude_deg <<" deg"
					<< std::endl;

			    });

      while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
      }

      ActionResult arm_result = action->arm();
      if(arm_result != ActionResult::SUCCESS){
	std::cout << "Arming failed: "
		  << action_result_str(arm_result)
		  << std::endl;

      }
      const std::string offb_mode = "BODY";


      int ch;

      initscr();

      halfdelay(3);

      noecho();

      keypad(stdscr, TRUE);

      while (true) {

	ch = getch();
	
	switch (ch) {
	  
	case KEY_UP:
	  std::cout<< "Move forward" << std::endl;
	  move_x(offboard);
	  
	  break;
	  
	case KEY_LEFT:
	//	move_y(offboard);
	  
	  break;
	case KEY_DOWN:
	  
	  break;
	case KEY_RIGHT:
	  std::cout<< "Move Right" << std::endl;
	  move_y(offboard);
	  break;
	  
	case 't':
	  takeoff(action);
	  sleep_for(seconds(6));
	  
	break;
	
	case 'l':
	  land(action);
	  break;
	  
	case 'o':
	  
	  break;
	  
	case 'p':
	  break;
	  
	default:
	  std::cout << "Unkown charcter has been pressed" << std::endl;
	}
      }
      
      // takeoff(action);
      
      // sleep_for(seconds(5));

      // offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

      // Offboard::Result offboard_result =  offboard->start();
      // offboard_error_exit(offboard_result, "Offboard start failed: ");
      // offboard_log(offb_mode, "Offboard started");


      // offboard_result = offboard->stop();
      // offboard_error_exit(offboard_result, "Offboard stop failed: ");
      // offboard_log(offb_mode, "Offboard stopped");
      clrtoeol();
      refresh();
      endwin();

      // land(action);

      sleep_for(seconds(10));
      std::cout << "Landed" << std::endl;

      return EXIT_SUCCESS;

}
