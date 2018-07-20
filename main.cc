/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-07-1
 */
extern "C"
{
  # include <curses.h>         
}

# include <string>
# include <iostream>
# include <boost/asio/posix/stream_descriptor.hpp> 
# include <boost/asio.hpp>

# include "controller.hh"



using namespace dronecore;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;


// inline void action_error_exit(ActionResult result, const std::string &message)
// {
//     if (result != ActionResult::SUCCESS) {
//         std::cerr << ERROR_CONSOLE_TEXT << message << action_result_str(result)
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         exit(EXIT_FAILURE);
//     }
// }

// // Handles Offboard's result
// inline void offboard_error_exit(Offboard::Result result, const std::string &message)
// {
//     if (result != Offboard::Result::SUCCESS) {
//         std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         exit(EXIT_FAILURE);
//     }
// }

// // Handles connection result
// inline void connection_error_exit(ConnectionResult result, const std::string &message)
// {
//     if (result != ConnectionResult::SUCCESS) {
//         std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         exit(EXIT_FAILURE);
//     }
// }


template <class Handler>
void async_wait(boost::asio::posix::stream_descriptor& in, Handler&& handler) {
  in.async_read_some(boost::asio::null_buffers(),
		     [&](boost::system::error_code ec,
			 size_t /* bytes_transferred */) {
		       if (not ec)
			 handler();
		       else
			   std::cerr << "Error: " << ec.message() << std::endl;
		       
			 async_wait(in, std::forward<Handler>(handler));
		     });
}


int main(int argc, char** argv)
{
  

  boost::asio::io_service	io_service;
  

  Controller controller;


  DroneCore dc_;
  
  controller.connect_to_quad(dc_, "udp://:14540");
  controller.discover_system(dc_);  

  
  // Wait for the system to connect via heartbeat
  // while (!dc_.is_connected()) {
  //   std::cout << "Wait for system to connect via heartbeat" << std::endl;
  //   sleep_for(seconds(1));
  // }
  
  //dc_.register_on_discover(event_callback_t callback)
  
  System& system = dc_.system();
  
  auto telemetry_ = std::make_shared<dronecore::Telemetry>(system);
  auto offboard_  = std::make_shared<dronecore::Offboard>(system);
  auto action_    = std::make_shared<dronecore::Action>(system);
    
  controller.set_rate_result(telemetry_);
  controller.get_position(telemetry_);
  controller.quad_health(telemetry_);

  


    
  setlocale(LC_ALL, "");
  
  initscr();
  
  halfdelay(3);
  
  
  // Suppress automatic echoing
  noecho();
  
  // Do not translate the return key into newline
  nonl();
  
  // Capture special keystrokes (including the four arrow keys)
  keypad(stdscr, TRUE);
  refresh();
  
  int ch;
  
  boost::asio::posix::stream_descriptor in{io_service, 0};
  //  in.assign(0); // stdscr
  
  auto lambda = [&](){
		  
		  ch = getch(); 
		  		  
		  switch (ch) {
		  case KEY_UP:
		    printw("key_up");
		    controller.forward(offboard_);
		    break;
		  case KEY_DOWN:
		    printw("key_down");
		    controller.backward(offboard_);
		    break;
		  case KEY_LEFT:
		    printw("key_left");
		    controller.goLeft(offboard_);
		    break;
		  case KEY_RIGHT:
		    printw("key_right");
		    controller.goRight(offboard_);
		    break;
		  case 't':
		    printw("take_off");
		    controller.takeoff(action_);
		    break;
		  case 'l':
		    printw("land");
		    controller.land(action_);
		    break;
		  case 'a':
		    controller.arm(action_);
		    sleep_for(seconds(2));
		    break;

		  case 's':
		    // Start offboard mode.

		    break;
		    
		  default:		    
		    printw("key_NOT DEFINED: %c", ch);
		    endwin();
		  }		  

		};
    
  async_wait(in, lambda);
  
  io_service.run();
             
}
