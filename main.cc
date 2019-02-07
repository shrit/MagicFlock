/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * 
 */

/*  C++ Standard library include */
# include <chrono>
# include <cstdlib>
# include <future>
# include <string>3
# include <thread>
# include <vector>

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


namespace lt = local_types;


/*
 * Wait for Joystick input, non-blocking implementation using STL 
*/

 JoystickEvent event_handler(Joystick& joystick, JoystickEvent event)
{
  
  while (true)
  {            
    // Attempt to read an event from the joystick
    JoystickEvent event;

    if (joystick.read_event(&event)){
      
      if(joystick.ButtonAChanged(event)) {
	std::cout << "A" << std::endl;			 
      }
      else if(joystick.ButtonBChanged(event)) {
	std::cout << "B" << std::endl;			 
      }
      else if(joystick.ButtonXChanged(event)) {
	std::cout << "X" << std::endl;			 
      }
      else if(joystick.ButtonYChanged(event)) {
	std::cout << "Y" << std::endl;			 
      }
      else if(joystick.ButtonL1Changed(event)) {
	std::cout << "L1" << std::endl;			 
      }
      else if(joystick.ButtonR1Changed(event)) {
	std::cout << "R1" << std::endl;			 
      }
      else if(joystick.ButtonSelectChanged(event)) {
	std::cout << "Select" << std::endl;			 
      }
      else if(joystick.ButtonStartChanged(event)) {
	std::cout << "Start" << std::endl;			 
      }     
      else if(joystick.ButtonGuideChanged(event)) {
	std::cout << "Guide" << std::endl;			 
      }
      else if(joystick.RightAxisXChanged(event)) {
	std::cout << "Rx: " << joystick.RightAxisXChanged(event) << std::endl;	 
      }
      else if (joystick.RightAxisYChanged(event)) {
	std::cout << "Ry: " << joystick.RightAxisYChanged(event) << std::endl;	 
      }
      else if (joystick.LeftAxisXChanged(event)) {
	std::cout << "Lx: " << joystick.LeftAxisXChanged(event) << std::endl;
      }
     else if (joystick.LeftAxisYChanged(event)) {
	std::cout << "Ly: " << joystick.LeftAxisYChanged(event) << std::endl;
     }
     else if (joystick.AxisL2Changed(event)) {
	std::cout << "L2: " << joystick.AxisL2Changed(event) << std::endl;
     }
     else if (joystick.AxisR2Changed(event)) {
       std::cout << "R2: " << joystick.AxisR2Changed(event) << std::endl;
     }
     else if (joystick.DpadXChanged(event)) {
       std::cout << "Dx: " << joystick.DpadXChanged(event) << std::endl;
     }
     else if (joystick.DpadYChanged(event)) {
       std::cout << "Dy: " << joystick.DpadYChanged(event) << std::endl;
     }      
    }       
  }
}

/*  Main file: Start one controller by quadcopters, 
 *  
 */

int main(int argc, char* argv[])
{

  /*  Calling the init logging file */
  /*  to be encapsulated in a classes */
  
  init();

  logging::add_common_attributes();
  
  boost::log::sources::severity_logger<level> lg;

  Joystick joystick("/dev/input/js0");
  
  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
    {
      std::cout << "No device found, please connect a joystick" << std::endl;
      exit(1);
    }
  
  JoystickEvent event;
  
  
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
  
  std::vector<std::shared_ptr<Px4Device>> iris_x;  
  
  for(auto& it : ports){					       
    								      
    iris_x.push_back(std::make_shared<Px4Device>("udp", it)); 
    std::cout  << "create an iris device" << std::endl;	       
  }								      

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

  gz->publisher("/gazebo/default/iris_1/model_reset");
  gz->publisher("/gazebo/default/iris_2/model_reset");
  gz->publisher("/gazebo/default/iris_3/model_reset");


  // gz->publisher("/gazebo/default/iris_opt_flow/model_reset");
  // gz->publisher("/gazebo/default/iris_opt_flow_1/model_reset");
  // gz->publisher("/gazebo/default/iris_opt_flow_2/model_reset");

  
  /* Wait for 2 seconds, Just to finish subscribe to
  * gazebo topics before Starting Q learning*/
  /*  to be reset 10 seconds before generating data set */
  std::this_thread::sleep_for(std::chrono::seconds(10));
  
  ////////////////
  // Q_learning //
  ////////////////

 
  //  std::cout <<   data_set.data_set() << std::endl;
  
  // Pass the devices to the q learning algorithm
  if(settings.train()) {
    DataSet data_set;
    Q_learning qlearning(iris_x, speed, gz, data_set);
    
  }

  arma::mat qtable;
  
  qtable.load("qtable_test");
  
  gz->rssi();

  double  maxi = arma::max(qtable(state));        

  

  
  ////////////////
  // Perceptron //
  ////////////////

    //  data_set.read_data_set_file("data_sample");      
  
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
  


  auto update_handler = [&](){			 

			  /**Update signal strength here 
			     other wise update it an other function 
			     each unit of time
			   */

			  
			  event_handler(joystick, event);
			};
  
  auto events =  std::async(std::launch::async, update_handler);
  
  
  events.get();
  
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
