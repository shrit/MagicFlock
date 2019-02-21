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
# include <string>
# include <thread>
# include <vector>

/*  locale defined include */

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

JoystickEvent event_handler(Joystick& joystick,
			    JoystickEvent event,
			    std::vector<std::shared_ptr<Px4Device>> iris_x,
			    float speed,
			    Q_learning qlearning,
			    arma::mat qtable,
			    std::shared_ptr<Gazebo> gz)
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
	
	std::cout << "arming..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      else if(joystick.ButtonBChanged(event)) {
	iris_x.at(0)->land();
	iris_x.at(1)->land();
	iris_x.at(2)->land();
	std::cout << "landing..." << std::endl;			 
      }
      else if(joystick.ButtonXChanged(event)) {

	iris_x.at(0)->takeoff();
	iris_x.at(1)->takeoff();
	iris_x.at(2)->takeoff();
	std::cout << "taking off..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(5));
      }
      else if(joystick.ButtonYChanged(event)) {
	
	iris_x.at(0)->init_speed();	
	iris_x.at(0)->start_offboard_mode();
	iris_x.at(1)->init_speed();	
	iris_x.at(1)->start_offboard_mode();
	iris_x.at(2)->init_speed();	
	iris_x.at(2)->start_offboard_mode();	
	std::cout << "Start offoard mode..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));          
	
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
	
	if(joystick.RightAxisXChanged(event) > 0 ){
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->right(speed);
	  std::cout << "Moving right... " << std::endl;
	}
	else{
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->left(speed);
	  std::cout << "Moving left... " << std::endl;
	}	
      }
      else if (joystick.RightAxisYChanged(event)) {        
	
	if(joystick.RightAxisYChanged(event) > 0 ){
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->backward(speed);
	  std::cout << "Moving backward... " << std::endl;	  	  
	}
	else{
	  /* Speed should be function of the value of joystick  */
	  /*  Speed should be fixed as the joystick does not move */
	  iris_x.at(0)->forward(speed);	  
	  std::cout << "Moving forward... " << std::endl;
	}				
      }
      else if (joystick.LeftAxisXChanged(event)) {

	if(joystick.LeftAxisXChanged(event) > 0 ){

	  iris_x.at(0)->turnToLeft(speed);		    
	  std::cout << "Turn to left... " <<std::endl;	  
	}
	else{
	  iris_x.at(0)->turnToRight(speed);		    
	  std::cout << "Turn to right... " <<std::endl;
	}		
      }
     else if (joystick.LeftAxisYChanged(event)) {

       if(joystick.LeftAxisYChanged(event) > 0 ){
	 /* Speed should be function of the value of joystick  */
	 iris_x.at(0)->down(speed);
	 std::cout << "Moving down...: " <<std::endl;	 	
       }
       else{
	 /* Speed should be function of the value of joystick  */
	 iris_x.at(0)->up(speed);	 
	 std::cout << "Moving up...: " <<std::endl;
       }	 
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
    //Moving other quads to this area
    
    std::cout << gz->rssi() << std::endl;
    arma::uword index =0;
    index = qlearning.qtable_state(gz, false);
    std::cout << "index: "<< index << std::endl;
    
    if (index > qtable.n_rows)
      {
	std::cout << "Move the leader around..."<< std::endl;
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

  /*  verify the numbers to subscribe to  the good signal strength*/
  
  gz->subscriber("/gazebo/default/1/2");
  gz->subscriber("/gazebo/default/1/3");
  gz->subscriber("/gazebo/default/2/3");

  gz->publisher("/gazebo/default/iris_1/model_reset");
  gz->publisher("/gazebo/default/iris_2/model_reset");
  gz->publisher("/gazebo/default/iris_3/model_reset");

  
  /* Wait for 10 seconds, Just to finish subscribe to
  * gazebo topics before Starting Q learning*/
  
  std::this_thread::sleep_for(std::chrono::seconds(10));
  
  ////////////////
  // Q_learning //
  ////////////////
  
  
  // Pass the devices to the q learning algorithm
  if(settings.train()) {
    DataSet data_set;
    Q_learning qlearning(iris_x, speed, gz, data_set, settings.train());
    return 0;
  }
  

  arma::mat qtable;
  DataSet data_set;
  Q_learning qlearning(iris_x, speed, gz, data_set, false);    

  bool ok = qtable.load("qtable_good");
  
  if(ok == false)
    {
      std::cout << "problem with loading the qtable" << std::endl;
    }

  /*  Add the keybord ncurses also cloe to the joystick */
  
  auto update_handler = [&](){			 
    
    /** Update signal strength here 
	other wise update it an other function 
	each unit of time    
	* use cantor get the index
	* move the quadcopters according to the action in the qtable */

    event_handler(joystick, event, iris_x, speed, qlearning, qtable, gz);
    
  };
  
  auto events =  std::async(std::launch::async, update_handler);
  
  
  events.get();
  
  
}
