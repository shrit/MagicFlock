/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * 
 */

// extern "C"
// {
//   # include <curses.h>         
// }

/*  C++ Standard library include */
# include <cstdlib>
# include <string>
# include <future>
# include <chrono>
# include <thread>
# include <vector>

/*  Boost asio library include */

//# include <boost/asio/posix/stream_descriptor.hpp>
//# include <boost/asio.hpp>

/*  locale defined include */
# include "algo/perceptron.h"
# include "algo/q_learning.hh"
# include "data_set.h"
# include "dronecode_sdk/logging.h"
# include "gazebo.hh"
# include "global.hh"
# include "joystick/joystick.hh"
# include "px4_device.hh"
# include "settings.hh"


using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;

/**  
 * TODO: Recover all the info of the telemetry
 * TODO: print well all the output using ncurses
 * TODO: use boost log to create a log, and to create possible statistical output
 * TODO: No defautl values for settings every thing shiyld be entered manually
*/


/*
 * Wait for keyboard input, non-blocking implementation using asio
*/


/*
 * Wait for Joystick input, non-blocking implementation using STL 
*/


namespace lt = local_types;

// template <class Handler>
// void async_wait(boost::asio::posix::stream_descriptor& in, Handler&& handler) {
//   in.async_read_some(boost::asio::null_buffers(),
// 		     [&](boost::system::error_code ec,
// 			 size_t /* bytes_transferred */) {
// 		       if (not ec)
// 			 handler();
// 		       else
// 			   std::cerr << "Error: " << ec.message() << std::endl;
		       
// 			 async_wait(in, std::forward<Handler>(handler));
// 		     });
// }
 
/*  Main file: Start one controller by quadcopters, 
 *  Start ncurses to intercept keyboard keystrokes.
 */

int main(int argc, char* argv[])
{

  /*  Calling the init logging file */
  /*  to be encapsulated in a classes */
  
  init();

  logging::add_common_attributes();
  
  boost::log::sources::severity_logger<level> lg;
  
  Settings settings(argc, argv);
  
  //  boost::asio::io_service	io_service;  

  /*  
   * The ns3 Command commented inside the code, A good way to remember it :)
   */
  //  /meta/ns-allinone-3.29/ns-3.29/ && /meta/ns-allinone-3.29/ns-3.29/waf --run  \"triangolo --fMode=4 --workDir=/meta/ns-allinone-3.29/ns-3.29 --xmlFilename=/meta/Spider-pig/gazebo/ns3/ns3.world --radioRange=300 --numusers=3\"
  
    
  int size = settings.quad_number() ;

  float speed = settings.speed();
  
  std::vector<lt::port_type> ports  =  settings.quads_ports();
  
  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */
  
  // std::vector<std::shared_ptr<Px4Device>> iris_x;  
  
  // for(auto& it : ports){					       
    								      
  //   iris_x.push_back(std::make_shared<Px4Device>("udp", it)); 
  //   std::cout  << "create an iris device" << std::endl;	       
  // }								      

  std::cout<< ports << std::endl;

  
  ////////////
  // Gazebo //
  ///////////
   
  /*  gazebo local test */
  
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc,argv);
  
  gz->subscriber("/gazebo/default/pose/info");  
  gz->subscriber("/gazebo/default/0/1");
  gz->subscriber("/gazebo/default/0/2");
  gz->subscriber("/gazebo/default/1/2");

  gz->publisher("/gazebo/default/iris/model_reset");
  gz->publisher("/gazebo/default/iris_1/model_reset");
  gz->publisher("/gazebo/default/iris_2/model_reset");


  // gz->publisher("/gazebo/default/iris_opt_flow/model_reset");
  // gz->publisher("/gazebo/default/iris_opt_flow_1/model_reset");
  // gz->publisher("/gazebo/default/iris_opt_flow_2/model_reset");

  
  /* Wait for 2 seconds, Just to finish subscribe to
  * gazebo topics before Starting Q learning*/
  /*  to be reset 10 seconds before generating data set */
  std::this_thread::sleep_for(std::chrono::seconds(4));
  
  ////////////////
  // Q_learning //
  ////////////////

 
  //  std::cout <<   data_set.data_set() << std::endl;


  

  // Pass the devices to the q learning algorithm
  //  Q_learning qlearning(iris_x, speed, gz);


  ////////////////
  // Perceptron //
  ////////////////
  DataSet data_set;
  
  data_set.read_data_set_file("data_sample");
  
  data_set.data_set();

  Perceptron tron(data_set.data_set().size(), 0.1, 0.4, data_set.data_set());

  
  
  // // Testing one drones components
  
  // iris_x.at(0)->arm();
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // iris_x.at(0)->takeoff();
  // std::this_thread::sleep_for(std::chrono::seconds(5));

  // iris_x.at(0)->init_speed();

  // iris_x.at(0)->start_offboard_mode();

  //     std::this_thread::sleep_for(std::chrono::seconds(1));          
  
  // for (int i =0 ; i < 30; i++ ){
  // iris_x.at(0)->forward(10);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }

  // std::this_thread::sleep_for(std::chrono::seconds(1));          
  // iris_x.at(0)->land();

  // std::this_thread::sleep_for(std::chrono::seconds(10));          
  
  // gz->reset_models();

  // iris_x.at(0)->reboot();
  
  
  /////////////
  // ncurses //
  /////////////
  
  // setlocale(LC_ALL, "");
  
  // initscr();
  
  // halfdelay(3);
    
  // // Suppress automatic echoing
  // noecho();
  
  // // Do not translate the return key into newline
  // nonl();
  
  // // Capture special keystrokes (including the four arrow keys)
  // keypad(stdscr, TRUE);
  
  // refresh();
  
  // int ch;
  
  //  boost::asio::posix::stream_descriptor in{io_service, 0};  

  /* Hand control using keyboard, manual control of the quadcopters */

  
  auto lambda = [&](){
		  
		//   ch = getch(); 
		  		  
		//   switch (ch) {
		//   case KEY_UP:
		//     printw("key_up");
		//     iris_x[0]->forward(speed);
		//     break;
		//   case KEY_DOWN:
		//     printw("key_down");
		//     iris_x[0]->backward(speed);
		//     break;
		//   case KEY_LEFT:
		//     printw("key_left");
		//     iris_x[0]->goLeft(speed);
		//     break;
		//   case KEY_RIGHT:
		//     printw("key_right");
		//     iris_x[0]->goRight(speed);
		//     break;
		//   case 'u':    
		//     printw("goUp");
		//     iris_x[0]->goUp(speed);
		//     break;
		//   case 'd':
		//     printw("goDown");
		//     iris_x[0]->goDown(speed);
		//     break;		    
		//   case 't':
		//     printw("take_off");
		//     iris_x[0]->takeoff();
		//     break;
		//   case 'l':
		//     printw("land");
		//     iris_x[0]->land();
		//     break;
		//   case 'a':
		//     printw("arming->->->");
		//     iris_x[0]->arm();
		//     sleep_for(seconds(2));
		//     break;
		//   case '+':
		//     printw("turn to right");
		//     iris_x[0]->turnToRight(speed);		    
		//     break;
		//   case '-':
		//     printw("turn to left");
		//     iris_x[0]->turnToLeft(speed);		    
		//     break;		    
		//   case 's':
		//     iris_x[0]->init_speed();
		//     iris_x[0]->start_offboard_mode();
		//     break;
		    
		//   default:		    
		//     printw("key_NOT DEFINED: %c", ch);
		//     endwin();
		//   }		  

		 };
    
  //  async_wait(in, lambda);

  //  io_service.run();
             
}
