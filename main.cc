/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * 
 */
extern "C" {
# include <curses.h>
}

/*  C++ Standard library include */
# include <chrono>
# include <cstdlib>
# include <future>
# include <string>
# include <thread>
# include <vector>

/*  locale defined include */

# include "algo/q_learning.hh"
# include "config_ini.hh"
# include "data_set.h"
# include "log.hh"
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

JoystickEvent joystick_event_handler(Joystick& joystick,
			    JoystickEvent event,
			    std::vector<std::shared_ptr<Px4Device>> iris_x,
			    float speed,
			    Q_learning qlearning,
			    arma::mat qtable,
			    std::shared_ptr<Gazebo> gz,
			    std::unordered_map<int, int> map)
{
  
  while (true)
  {            
    // Attempt to read an event from the joystick
    JoystickEvent event;

    if (joystick.read_event(&event)){
      
      if(joystick.ButtonAChanged(event)) {
	iris_x.at(0)->arm();
	iris_x.at(1)->arm();
	iris_x.at(2)->arm();
	
        LogInfo() << "arming...";
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      else if(joystick.ButtonBChanged(event)) {
	iris_x.at(0)->land();
	iris_x.at(1)->land();
	iris_x.at(2)->land();
	LogInfo() << "landing..." ;			 
      }
      else if(joystick.ButtonXChanged(event)) {

	iris_x.at(0)->takeoff();
	iris_x.at(1)->takeoff();
	iris_x.at(2)->takeoff();
	LogInfo() << "taking off..." ;
	std::this_thread::sleep_for(std::chrono::seconds(5));
      }
      else if(joystick.ButtonYChanged(event)) {
	
	iris_x.at(0)->init_speed();	
	iris_x.at(0)->start_offboard_mode();
	iris_x.at(1)->init_speed();	
	iris_x.at(1)->start_offboard_mode();
	iris_x.at(2)->init_speed();	
	iris_x.at(2)->start_offboard_mode();	
	LogInfo() << "Start offoard mode..." ;
	std::this_thread::sleep_for(std::chrono::seconds(1));          
	
      }
      else if(joystick.ButtonL1Changed(event)) {
	LogInfo() << "L1" ;			 
      }
      else if(joystick.ButtonR1Changed(event)) {
	LogInfo() << "R1" ;			 
      }
      else if(joystick.ButtonSelectChanged(event)) {
	LogInfo() << "Select" ;			 
      }
      else if(joystick.ButtonStartChanged(event)) {
	LogInfo() << "Start" ;			 
      }     
      else if(joystick.ButtonGuideChanged(event)) {
	LogInfo() << "Guide" ;			 
      }
      else if(joystick.RightAxisXChanged(event)) {
	
	if(joystick.RightAxisXChanged(event) > 0 ){
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->right(speed);
	  LogInfo() << "Moving right... " ;
	}
	else{
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->left(speed);
	  LogInfo() << "Moving left... " ;
	}	
      }
      else if (joystick.RightAxisYChanged(event)) {        
	
	if(joystick.RightAxisYChanged(event) > 0 ){
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->backward(speed);
	  LogInfo() << "Moving backward... " ;	  	  
	}
	else{
	  /* Speed should be function of the value of joystick  */
	  /*  Speed should be fixed as the joystick does not move */
	  iris_x.at(0)->forward(speed);	  
	  LogInfo() << "Moving forward... " ;
	}				
      }
      else if (joystick.LeftAxisXChanged(event)) {

	if(joystick.LeftAxisXChanged(event) > 0 ){

	  iris_x.at(0)->turnToLeft(speed);		    
	  LogInfo() << "Turn to left... " ;	  
	}
	else{
	  iris_x.at(0)->turnToRight(speed);		    
	  LogInfo() << "Turn to right... " ;
	}		
      }
     else if (joystick.LeftAxisYChanged(event)) {

       if(joystick.LeftAxisYChanged(event) > 0 ){
	 /* Speed should be function of the value of joystick  */
	 iris_x.at(0)->down(speed);
	 LogInfo() << "Moving down...: " ;	 	
       }
       else{
	 /* Speed should be function of the value of joystick  */
	 iris_x.at(0)->up(speed);	 
	 LogInfo() << "Moving up...: " ;
       }	 
     }
     else if (joystick.AxisL2Changed(event)) {
	LogInfo() << "L2: " << joystick.AxisL2Changed(event) ;
     }
     else if (joystick.AxisR2Changed(event)) {
       LogInfo() << "R2: " << joystick.AxisR2Changed(event) ;
     }
     else if (joystick.DpadXChanged(event)) {
       LogInfo() << "Dx: " << joystick.DpadXChanged(event) ;
     }
     else if (joystick.DpadYChanged(event)) {
       LogInfo() << "Dy: " << joystick.DpadYChanged(event) ;
     }      
    }
    //Moving other quads to this area
    
    LogInfo() << gz->rssi() ;
    arma::uword index =0;
   
    index = qlearning.qtable_state_from_map(gz, map);
    LogInfo() << "index: "<< index ;    
    
    if (index > qtable.n_rows)
      {
	LogInfo() << "Move the leader around...";
      }
    else {
      qlearning.qtable_action(qtable, index);
    }            
    
  }
}


void keyboard_event_handler(std::vector<std::shared_ptr<Px4Device>> iris_x,
			    float speed,
			    Q_learning qlearning,
			    arma::mat qtable,
			    std::shared_ptr<Gazebo> gz,
			    std::unordered_map<int, int> map)
{
  int ch;
  
  while (true)
    {            
      // Attempt to read an event from the joystick

      ch = getch();
      
      switch (ch) {
	
      case 'a':
	iris_x.at(0)->arm();
	iris_x.at(1)->arm();
	iris_x.at(2)->arm();	
	LogInfo() << "arming..." ;

      case 'l':
	iris_x.at(0)->land();
	iris_x.at(1)->land();
	iris_x.at(2)->land();
	LogInfo() << "landing..." ;
	
      case 't':
	iris_x.at(0)->takeoff();
	iris_x.at(1)->takeoff();
	iris_x.at(2)->takeoff();
	LogInfo() << "taking off..." ;
	std::this_thread::sleep_for(std::chrono::seconds(5));

      case 's':
	iris_x.at(0)->init_speed();	
	iris_x.at(0)->start_offboard_mode();
	iris_x.at(1)->init_speed();	
	iris_x.at(1)->start_offboard_mode();
	iris_x.at(2)->init_speed();	
	iris_x.at(2)->start_offboard_mode();	
	LogInfo() << "Start offoard mode..." ;
	std::this_thread::sleep_for(std::chrono::seconds(1));          

      case KEY_RIGHT:	
	iris_x.at(0)->right(speed);
	LogInfo() << "Moving right... " ;

      case KEY_LEFT:	
	iris_x.at(0)->left(speed);
	LogInfo() << "Moving left... " ;

      case KEY_DOWN:	
	iris_x.at(0)->backward(speed);
	LogInfo() << "Moving backward... " ;	  	  
	
      case KEY_UP: 
	/*  Speed should be fixed as the joystick does not move */
	iris_x.at(0)->forward(speed);	  
	LogInfo() << "Moving forward... " ;
		
      case 'd':
	iris_x.at(0)->down(speed);
	LogInfo() << "Moving down...: " ;

      case 'u': 	
	iris_x.at(0)->up(speed);	 
	LogInfo() << "Moving up...: " ;
      default:		    	
	printw("key_NOT DEFINED: %c", ch);	
	endwin();	
      }	
      	 
    LogInfo() << gz->rssi() ;
    arma::uword index =0;
   
    index = qlearning.qtable_state_from_map(gz, map);
    LogInfo() << "index: "<< index ;    
    
    if (index > qtable.n_rows)
      {
	LogInfo() << "Move the leader around...";
      }
    else {
      qlearning.qtable_action(qtable, index);
    }                
  }
}



/*  Main file: Start one controller by quadcopters, 
 *  
 */

int main(int argc, char* argv[])
{


  Joystick joystick("/dev/input/js0");
  
  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
    {
      LogInfo() << "No device found, please connect a joystick" ;
      exit(1);
    }
  
  JoystickEvent event;
    
  Settings settings(argc, argv);

  Configs configs;
  
  /*  
   * The ns3 Command commented inside the code, A good way to remember it :)
   */
  //  /meta/ns-allinone-3.29/ns-3.29/ && /meta/ns-allinone-3.29/ns-3.29/waf --run  \"triangolo --fMode=4 --workDir=/meta/ns-allinone-3.29/ns-3.29 --xmlFilename=/meta/Spider-pig/gazebo/ns3/ns3.world --radioRange=300 --numusers=3\"
  
    
  int size = configs.quad_number() ;

  float speed = configs.speed();
  
  std::vector<lt::port_type> ports  =  configs.quads_ports();
  
  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */


  //chagnge iris into a string and capture from ini file
  std::vector<std::shared_ptr<Px4Device>> iris_x;  
  
  for(auto& it : ports){					       
    								      
    iris_x.push_back(std::make_shared<Px4Device>("udp", it)); 
    LogInfo()  << "create an iris device" ;	       
  }								      

  LogInfo()<< ports ;

  
  ////////////
  // Gazebo //
  ///////////
   
  /*  gazebo local test */
  
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc,argv);
  
  gz->subscriber(configs.positions());

  /*  verify the numbers to subscribe to the good signal strength*/
  
  gz->subscriber(configs.rssi_1_2());
  gz->subscriber(configs.rssi_1_3());
  gz->subscriber(configs.rssi_2_3());

  gz->publisher(configs.reset_1());
  gz->publisher(configs.reset_2());
  gz->publisher(configs.reset_3());

  
  /* Wait for 10 seconds, Just to finish subscribe to
  * gazebo topics before Starting Q learning*/
  
  std::this_thread::sleep_for(std::chrono::seconds(10));
  
  ////////////////
  // Q_learning //
  ////////////////
  
  
  // Pass the devices to the q learning algorithm
  if(configs.train()) {
    DataSet data_set;
    Q_learning qlearning(iris_x, speed, gz, data_set, configs.train());
    return 0;
  }
  
  arma::mat qtable;
  DataSet data_set;
  Q_learning qlearning(iris_x, speed, gz, data_set, false);    

  std::unordered_map<int, int> map;
  data_set.read_map_file(configs.map_file_name(), map);

  for(auto elem : map){
      LogInfo() << elem.first << " " << elem.second << "\n";
    }

  std::this_thread::sleep_for(std::chrono::seconds(5));  

  //"/data_set/qtable_6000"
  
  bool ok = qtable.load(configs.qtable_file_name());
  
  if(ok == false){
      LogWarn() << "problem with loading the qtable";
    }


  //Ncurses
  
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
  
  
  
  /*  Add the keybord ncurses also cloe to the joystick */
  
  auto joystick_handler = [&](){			 
    
			  /** Update signal strength here 
			      other wise update it an other function 
			      each unit of time    
			      * use cantor get the index
			      * move the quadcopters according to the action in the qtable */
			  
			  joystick_event_handler(joystick, event,
						 iris_x, speed,
						 qlearning, qtable,
						 gz, map);
    
  };


  
  auto keyboard_handler = [&](){			 
    
			  /** Update signal strength here 
			      other wise update it an other function 
			      each unit of time    
			      * use cantor get the index
			      * move the quadcopters according to the action in the qtable */
			  
			  keyboard_event_handler(iris_x, speed, qlearning, qtable, gz, map);
    
  };
  
  auto joystick_events =  std::async(std::launch::async, joystick_handler);
  auto keyboard_events =  std::async(std::launch::async, keyboard_handler);  
  
  joystick_events.get();
  keyboard_events.get();
    
}
