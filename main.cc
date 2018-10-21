/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-07-21
 */

extern "C"
{
  # include <curses.h>         
}

/*  C++ Standard library include */

# include <string>
# include <sstream>
# include <future>
# include <chrono>
# include <vector>

/*  Boost asio library include */

# include <boost/asio/posix/stream_descriptor.hpp>
# include <boost/asio.hpp>

/*  gazebo include */

# include <gazebo/gazebo_config.h>
# include <gazebo/transport/transport.hh>
# include <gazebo/gazebo_client.hh>
# include <gazebo/msgs/msgs.hh>

/*  locale defined include */
# include "controller.hh"
# include "dronecode_sdk/logging.h"
# include "settings.hh"

using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;

/**  
 * TODO: Comment all the code 
 * TODO: Manage the error exit
 * TODO: Recover all the info of the telemetry
 * TODO: print well all the output using ncurses
 * TODO: use boost log to create a log, and to create possible statistical output
 * TODO: the log sould be in different folder, created at the start 
 * TODO: Also understand the normal log provided by the library
 * TODO: Create a phase like principle in this code with the same content of robotsim
 * TODO: Implement camera receive video, start, and stop, take photo, etc..
 * TODO: Add a timer for data that are sent, also use timer in the client side
 * to ask for data each 50 ms
*/


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


/*
 * Wait for keyboard input, non-blocking implementation using asio
*/

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


/*  Main file: Start one controller by quadcopters, 
 *  Start ncurses to intercept keyboard keystrokes.
 */

int main(int argc, char* argv[])
{

  std::stringstream ss;

  Settings settings(argc, argv);
        
  boost::asio::io_service	io_service;

  /* Create a vector of controllers
   * Each controller connect to one quadcopters at a time
   * 
   */
  
  std::vector<Controller> controllers;

  
  for(auto i : controllers){
    Controller controller;
    DronecodeSDK dc_;
    //find a hack to pass port numbers to quads
    controller.connect_to_quad(dc_, settings.get_connection_url());
    
    controller.discover_system(dc_);  
    
    System& system = dc_.system();
    
    auto telemetry_ = std::make_shared<dronecode_sdk::Telemetry>(system);
    auto offboard_  = std::make_shared<dronecode_sdk::Offboard>(system);
    auto action_    = std::make_shared<dronecode_sdk::Action>(system);
    
    controller.set_rate_result(telemetry_);
    controller.print_position(telemetry_);
    controller.quad_health(telemetry_);    
    
    controllers.push_back(controller);
  }

  //Subscribe to gazebo topics published by Ns3

  ////////////
  // Gazebo /
  ///////////
  
  
  using SubPtr = gazebo::transport::SubscriberPtr
 
  gazebo::client::setup(argc, argv);
  
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  
  node->Init();

  // to subscribe to the 
  SubPtr sub = node->Subscribe("~/", function_to_call_treat_the_msg);  
 
  /////////////
  // ncurses //
  /////////////

  
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
      
  auto lambda = [&](){
		  
		  ch = getch(); 
		  		  
		  switch (ch) {
		  case KEY_UP:
		    printw("key_up");
		    controllers.at(0).forward(offboard_);
		    break;
		  case KEY_DOWN:
		    printw("key_down");
		    controllers.at(0).backward(offboard_);
		    break;
		  case KEY_LEFT:
		    printw("key_left");
		    controllers.at(0).goLeft(offboard_);
		    break;
		  case KEY_RIGHT:
		    printw("key_right");
		    controller.goRight(offboard_);
		    break;
		  case 'u':    
		    printw("goUp");
		    controllers.at(0).goUp(offboard_);
		    break;
		  case 'd':
		    printw("goDown");
		    controllers.at(0).goDown(offboard_);
		    break;		    
		  case 't':
		    printw("take_off");
		    controllers.at(0).takeoff(action_);
		    break;
		  case 'l':
		    printw("land");
		    controllers.at(0).land(action_);
		    break;
		  case 'a':
		    printw("arming...");
		    controllers.at(0).arm(action_);
		    sleep_for(seconds(2));
		    break;
		  case '+':
		    printw("turn to right");
		    controllers.at(0).turnToRight(offboard_);		    
		    break;
		  case '-':
		    printw("turn to left");
		    controllers.at(0).turnToLeft(offboard_);		    
		    break;		    
		  case 's':
		    controllers.at(0).init_speed(offboard_);
		    controllers.at(0).start_offboard_mode(offboard_);
		    break;
		    
		  default:		    
		    printw("key_NOT DEFINED: %c", ch);
		    endwin();
		  }		  

		};
    
  async_wait(in, lambda);

  io_service.run();
             
}
