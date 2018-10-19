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

//# include <ros/ros.h>
//# include <geometry_msgs/Pose.h>


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
 * TODO: Reput the code in several files URGENT, 
 * TODO: keep the ros commented in the main
 * TODO: Make the client send the position message to the server
 * TODO: Reimplement the server seperatly in a connection file 
 * TODO: Comment all the code 
 * TODO: Manage the error exit
 * TODO: Recover all the info of the telemetry
 * TODO: print well all the output using ncurses
 * TODO: use boost log to create a log, and to create possible statistical output
 * TODO: Also understand the normal log provided by the library
 * TODO: Implement camera receive video, start, and stop, take photo, etc..
 * TODO: Add a timer for data that are sent, also use timer in the client side
 * to ask for data each 50 ms
 * TODO: Try to use opengl instead of ncurses for keyboard command,
 * TODO: Use the opengl graphical interface
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


int main(int argc, char* argv[])
{

  std::stringstream ss;

  Settings settings(argc, argv);
        
  boost::asio::io_service	io_service;

  Controller controller;

  DronecodeSDK dc_;
  
  controller.connect_to_quad(dc_, settings.get_connection_url());

  controller.discover_system(dc_);  

  System& system = dc_.system();
  
  auto telemetry_ = std::make_shared<dronecode_sdk::Telemetry>(system);
  auto offboard_  = std::make_shared<dronecode_sdk::Offboard>(system);
  auto action_    = std::make_shared<dronecode_sdk::Action>(system);
    
  controller.set_rate_result(telemetry_);
  controller.print_position(telemetry_);
  controller.quad_health(telemetry_);
  controller.get_position_ned();

  //Connect to the server installed in ns3

  ////////////
  // Gazebo //
  ////////////


  using PubPtr =  gazebo::transport::PublisherPtr ;
  
 
  gazebo::client::setup(argc, argv);
  
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  
  node->Init();

  PubPtr pub = node->Advertise<gazebo::msgs::Vector3d>("~/quad_position/pose");

  // std::vector<double> msg;
  
  // msg.push_back(controller.get_position_ned().position.down_m);
  // msg.push_back(controller.get_position_ned().position.east_m);
  // msg.push_back(controller.get_position_ned().position.north_m);
  
  
  gazebo::msgs::Vector3d msg;
  gazebo::msgs::Set(&msg,
		    ignition::math::Vector3d(controller.get_position_ned().position.down_m,
					     controller.get_position_ned().position.east_m,
					     controller.get_position_ned().position.north_m));
  pub->Publish(msg);  
  
  ///////////
  // ROS   //
  ///////////
  
  //   ros::init(argc, argv, "node");
  
   //  ros::NodeHandle node_handle_;
  
   //    ros::Publisher pose_pub;
  
  //  using pose_msg = geometry_msgs::Pose;
    
    
  // pose_pub = node_handle_.advertise<pose_msg>("/" + ss.str() + "/pose", 1000);
    
    // pose_msg pose;							 
    // pose.position.x = controller.get_position_ned().position.down_m ;	
    // pose.position.y = controller.get_position_ned().position.east_m;
    // pose.position.z = controller.get_position_ned().position.north_m;   
    
    // pose_pub.publish(pose);
    
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
		  case 'u':    
		    printw("goUp");
		    controller.goUp(offboard_);
		    break;
		  case 'd':
		    printw("goDown");
		    controller.goDown(offboard_);
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
		    printw("arming...");
		    controller.arm(action_);
		    sleep_for(seconds(2));
		    break;
		  case '+':
		    printw("turn to right");
		    controller.turnToRight(offboard_);		    
		    break;
		  case '-':
		    printw("turn to left");
		    controller.turnToLeft(offboard_);		    
		    break;		    
		  case 's':
		    controller.init_speed(offboard_);
		    controller.start_offboard_mode(offboard_);
		    break;
		    
		  default:		    
		    printw("key_NOT DEFINED: %c", ch);
		    endwin();
		  }		  

		};
    
  async_wait(in, lambda);

  io_service.run();
             
}
