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
# include <future>
# include <chrono>
# include <vector>

/*  Boost asio library include */

# include <boost/asio/posix/stream_descriptor.hpp>
# include <boost/asio.hpp>

/*  gazebo include */
# include "gazebo.hh"

/*  locale defined include */
# include "controller.hh"
# include "dronecode_sdk/logging.h"
//# include "settings.hh"

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

 * TODO: Use boost.python to interface python to C++, for using numpy.
 * TODO: Complete the Q_learning algorithm, and the classes.
 * 
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

  Settings settings(argc, argv);
  
  boost::asio::io_service	io_service;
  
  /* Create a vector of controllers
   * Each controller connect to one quadcopters at a time
   * 
   */
  
  std::vector<Controller> controllers;
  
  for(auto i : controllers){
    
    Controller controller(settings);    
    controllers.push_back(controller);
  }

  //Subscribe to gazebo topics published by Ns3

  ////////////
  // Gazebo /
  ///////////

  Gazebo gazebo(argc, argv);
  
  gazebo.subscriber("/gazebo/default/pose/info");
    
  
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
		    controller.forward();
		    break;
		  case KEY_DOWN:
		    printw("key_down");
		    controller.backward();
		    break;
		  case KEY_LEFT:
		    printw("key_left");
		    controller.goLeft();
		    break;
		  case KEY_RIGHT:
		    printw("key_right");
		    controller.goRight();
		    break;
		  case 'u':    
		    printw("goUp");
		    controller.goUp();
		    break;
		  case 'd':
		    printw("goDown");
		    controller.goDown();
		    break;		    
		  case 't':
		    printw("take_off");
		    controller.takeoff();
		    break;
		  case 'l':
		    printw("land");
		    controller.land();
		    break;
		  case 'a':
		    printw("arming...");
		    controller.arm();
		    sleep_for(seconds(2));
		    break;
		  case '+':
		    printw("turn to right");
		    controller.turnToRight();		    
		    break;
		  case '-':
		    printw("turn to left");
		    controller.turnToLeft();		    
		    break;		    
		  case 's':
		    controller.init_speed();
		    controller.start_mode();
		    break;
		    
		  default:		    
		    printw("key_NOT DEFINED: %c", ch);
		    endwin();
		  }		  

		};
    
  async_wait(in, lambda);

  io_service.run();
             
}
