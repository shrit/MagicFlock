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
# include <cstdlib>
# include <string>
# include <future>
# include <chrono>
# include <vector>

/*  Boost asio library include */

# include <boost/asio/posix/stream_descriptor.hpp>
# include <boost/asio.hpp>


/*  locale defined include */
# include "gazebo.hh"
# include "px4_device.hh"
# include "global.hh"
# include "settings.hh"
# include "algo/q_learning.hh"
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
 * TODO: Use Google protobuf to parse the input
 * TODO: Use boost.python to interface python to C++, for using numpy.
 * TODO: Complete the Q_learning algorithm, and the classes.
 * TODO: No defautl values for settings every thing shiyld be entered manually
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


namespace lt = local_types;

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

  /*  Dirty Hack to run NS3 using the code, it needs professionals to
  * integrate the entire NS3 simulation code into our code. Thus we
  * are using this method */
  
  // int x = std::system("cd /meta/ns-allinone-3.29/ns-3.29/ && /meta/ns-allinone-3.29/ns-3.29/waf --run  \"triangolo --fMode=4 --workDir=/meta/ns-allinone-3.29/ns-3.29 --xmlFilename=/meta/Spider-pig/gazebo/ns3/ns3.world --radioRange=300 --numusers=3\"");
  // if (x == 0)
  //   std::cout << "we are in ns3 directory" << std::endl; 
  
  int size = settings.quad_number() ;

  float speed = settings.speed();
  
  std::vector<lt::port_type> ports  =  settings.quads_ports();
  
  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */

  
  std::vector<std::shared_ptr<Controller>> controllers;  
  
  for(auto& it : ports){					       
    								      
    controllers.push_back(std::make_shared<Controller>("udp", it)); 
    std::cout  << "create a controller" << std::endl;	       
  }								      
   
  //Subscribe to gazebo topics published by NS3
  
  ////////////
  // Gazebo /
  ///////////

  Gazebo gazebo(argc, argv);
  
  gazebo.subscriber("/gazebo/default/pose/info");
    

  ////////////////
  // Q_learning //
  ////////////////
  
  //  Q_learning qlearning;
  
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
		    controllers[0]->forward(speed);
		    break;
		  case KEY_DOWN:
		    printw("key_down");
		    controllers[0]->backward(speed);
		    break;
		  case KEY_LEFT:
		    printw("key_left");
		    controllers[0]->goLeft(speed);
		    break;
		  case KEY_RIGHT:
		    printw("key_right");
		    controllers[0]->goRight(speed);
		    break;
		  case 'u':    
		    printw("goUp");
		    controllers[0]->goUp(speed);
		    break;
		  case 'd':
		    printw("goDown");
		    controllers[0]->goDown(speed);
		    break;		    
		  case 't':
		    printw("take_off");
		    controllers[0]->takeoff();
		    break;
		  case 'l':
		    printw("land");
		    controllers[0]->land();
		    break;
		  case 'a':
		    printw("arming->->->");
		    controllers[0]->arm();
		    sleep_for(seconds(2));
		    break;
		  case '+':
		    printw("turn to right");
		    controllers[0]->turnToRight(speed);		    
		    break;
		  case '-':
		    printw("turn to left");
		    controllers[0]->turnToLeft(speed);		    
		    break;		    
		  case 's':
		    controllers[0]->init_speed();
		    controllers[0]->start_offboard_mode();
		    break;
		    
		  default:		    
		    printw("key_NOT DEFINED: %c", ch);
		    endwin();
		  }		  

		};
    
  async_wait(in, lambda);

  io_service.run();
             
}
