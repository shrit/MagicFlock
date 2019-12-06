/**
 * @file main.cc 
 * @brief allow user to control the quad using joystick
 * and keyboard
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

# include <ILMR/config_ini.hh>

# include <ILMR/global.hh>
# include <ILMR/joystick.hh>
# include <ILMR/keyboard.hh>
# include <ILMR/log.hh>
# include <ILMR/px4_device.hh>

namespace lt = local_types;

/*
 * Wait for Joystick input, non-blocking implementation using STL
*/
JoystickEvent joystick_event_handler(Joystick& joystick,
				     std::vector<std::shared_ptr<Px4Device>> iris_x,
				     float speed,
				     bool just_fly)
{
  while (true) {
    // Attempt to read an event from the joystick
    JoystickEvent event;    
    if (joystick.read_event(&event)) {
      
      if (joystick.ButtonAChanged(event)) {
	if (!just_fly) {
	  for (auto it : iris_x) {
	    it->arm();
	  }
	} else {
	  iris_x.at(0)->arm();
	}
        LogInfo() << "arming...";
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
      } else if (joystick.ButtonBChanged(event)) {
	if (!just_fly) {
	  for (auto it : iris_x) {
	    it->land();
	  }
	} else {
	  iris_x.at(0)->land();
	}
	LogInfo() << "landing...";
	
      } else if (joystick.ButtonXChanged(event)) {
	if (!just_fly) {
	  for (auto it : iris_x) {
	    it->takeoff();
	  }
	} else {
	  iris_x.at(0)->takeoff();
	}
	LogInfo() << "taking off...";
	std::this_thread::sleep_for(std::chrono::seconds(5));
	
      } else if (joystick.ButtonYChanged(event)) {
	if (!just_fly) {
	  for (auto it : iris_x) {
	    it->init_speed();
	  }
	  for (auto it : iris_x) {
	    it->start_offboard_mode();
	  }
	} else {
	  iris_x.at(0)->init_speed();
	  iris_x.at(0)->start_offboard_mode();
	}
	
	LogInfo() << "Start offoard mode...";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	
      } else if (joystick.ButtonL1Changed(event)) {
	LogInfo() << "L1";
	
      } else if (joystick.ButtonR1Changed(event)) {
	LogInfo() << "R1";
	
      } else if (joystick.ButtonSelectChanged(event)) {
	LogInfo() << "Select";
	
      } else if (joystick.ButtonStartChanged(event)) {
	LogInfo() << "Start";
	
      } else if (joystick.ButtonGuideChanged(event)) {
	LogInfo() << "Guide";
	
      } else if (joystick.RightAxisXChanged(event)) {
	
	if (joystick.RightAxisXChanged(event) > 0 ) {
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->right(speed);
	  LogInfo() << "Moving right... ";
	  
	} else {
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->left(speed);
	  LogInfo() << "Moving left... ";
	}
      } else if (joystick.RightAxisYChanged(event)) {
	
	if (joystick.RightAxisYChanged(event) > 0 ) {
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->backward(speed);
	  LogInfo() << "Moving backward... ";
	  
	} else {
	  /* Speed should be function of the value of joystick  */
	  /*  Speed should be fixed as the joystick does not move */
	  iris_x.at(0)->forward(speed);
	  LogInfo() << "Moving forward... " ;
	}
	
      } else if (joystick.LeftAxisXChanged(event)) {
	
	if (joystick.LeftAxisXChanged(event) > 0 ) {
	  
	  iris_x.at(0)->turnToLeft();
	  LogInfo() << "Turn to left... " ;
	  
	} else {
	  iris_x.at(0)->turnToRight();
	  LogInfo() << "Turn to right... " ;
	}
	
      } else if (joystick.LeftAxisYChanged(event)) {
	
	if (joystick.LeftAxisYChanged(event) > 0 ){
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->down(speed);
	  LogInfo() << "Moving down...: " ;
	  
	} else {
	  /* Speed should be function of the value of joystick  */
	  iris_x.at(0)->up(speed);
	  LogInfo() << "Moving up...: " ;
	}
      } else if (joystick.AxisL2Changed(event)) {
	LogInfo() << "L2: " << joystick.AxisL2Changed(event);
	
      } else if (joystick.AxisR2Changed(event)) {
	LogInfo() << "R2: " << joystick.AxisR2Changed(event);
	
      } else if (joystick.DpadXChanged(event)) {
	LogInfo() << "Dx: " << joystick.DpadXChanged(event);
	
      } else if (joystick.DpadYChanged(event)) {
	LogInfo() << "Dy: " << joystick.DpadYChanged(event);
      }
    }
  }
}

void keyboard_event_handler(std::vector<std::shared_ptr<Px4Device>> iris_x,
			    float speed,
			    bool just_fly)
{
  Keyboard keyboard(STDIN_FILENO);
  while (true) {
      // Attempt to read an event from the joystick
    int ch = keyboard.poll_event(STDIN_FILENO);

    switch (ch) {      
    case 'm':
      if (!just_fly) {
	for (auto it : iris_x) {
	  it->arm();
	}
      } else {
	iris_x.at(0)->arm();
      }
      LogInfo() << "Arming...";
      break;
    case 'l':
      if (!just_fly) {
	for (auto it : iris_x) {
	  it->land();
	}
      } else {
	iris_x.at(0)->land();
      }      
      LogInfo() << "Landing...";
      break;
    case 't':
      if (!just_fly) {
	for (auto it : iris_x) {
	  it->takeoff();
	}
      } else {
	iris_x.at(0)->takeoff();
      }
      LogInfo() << "Taking off...";
      std::this_thread::sleep_for(std::chrono::seconds(5));      
      break;
    case 'o':
      if (!just_fly) {
	for (auto it : iris_x) {
	  it->init_speed();
	}
	for (auto it : iris_x) {
	  it->start_offboard_mode();
	}
      } else {
	iris_x.at(0)->init_speed();
	iris_x.at(0)->start_offboard_mode();
      }      
      LogInfo() << "Start offoard mode...";
      std::this_thread::sleep_for(std::chrono::seconds(1));
      break;
    case 'd':
      iris_x.at(0)->right(speed);
      LogInfo() << "Moving right...";
      break;
    case 'a':
      iris_x.at(0)->left(speed);
      LogInfo() << "Moving left...";
      break;
    case 's':
      iris_x.at(0)->backward(speed);
      LogInfo() << "Moving backward... ";
      break;
    case 'w':
      iris_x.at(0)->forward(speed);
      LogInfo() << "Moving forward... ";
      break;
    case 'z':
      iris_x.at(0)->down(speed);
      LogInfo() << "Moving down...: ";
      break;
    case 'q':
      iris_x.at(0)->up(speed);
      LogInfo() << "Moving up...: ";
      break;
    case static_cast<int>(Keyboard::Special_keys::CTRL_C):
      keyboard.disable_raw_mode(STDIN_FILENO);
      exit(0);
    default:
      LogInfo() << "NON assigned key: " << ch;
      break;
    }
  }
}

/*  Print the possible keyboard or joystick input */
void usage(std::ostream& out)
{

  out<< "Usage : " << std ::endl
     << "To fly a quadrotor manually you need to start the software with flying option" << std::endl
     << "To control the quadcopter using keyboard use : " << std::endl
     << " m : arm" << std::endl
     << " t : takoff" << std::endl
     << " l : land" << std::endl
     << " o : activate offboard mode" << std::endl
     << " w : to go forward" << std::endl
     << " d : to go right" << std::endl
     << " a : to go left" << std::endl
     << " s : to go backward" << std::endl
     << " + : to turn clock wise" << std::endl
     << " - : to turn counter clock wise" << std::endl
     << "To control the quadcopter using Xbox joystick use : " << std::endl
     << " A : to arm" << std::endl
     << " X : takeoff "<< std::endl
     << " Y : activate offboard mode" << std::endl
     << " B : land" << std::endl
     << " Right Stick up    : to go forward" << std::endl
     << " Right Stick right : to go right" << std::endl
     << " Right Stick left : to go left" << std::endl
     << " Right Stick down : to go backword" << std::endl
     << " Left Stick up    : to go up" << std::endl
     << " Left Stick right : to go down" << std::endl;
}

/*
 *  Main file: Start one controller by quadcopters
 */
int main(int argc, char* argv[])
{

  Configs configs;
  
  /*  Init logging system */
  Log log;
  log.init();

  bool joystick_mode = true;
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound()) {
    LogErr() << "No device found, please connect a joystick" ;
    LogErr() << "Joystick mode disabled. Control only possible using a keyboard";
    joystick_mode = false;
  }
  
  std::vector<lt::port_type> ports = configs.quads_ports();
  
  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */
  std::vector<std::shared_ptr<Px4Device>> iris_x;
  for (auto& it : ports) {
    iris_x.push_back(std::make_shared<Px4Device>("udp", it));
    LogInfo() << "Add an iris QCopter! " ;
  }

  LogInfo() << "Ports number: "<< ports;     
  std::shared_ptr<Gazebo> gz = std::make_shared<Gazebo>(argc,argv);

  gz->subscriber(configs.positions());

  /* Verify the numbers to subscribe to the good signal strength */
  gz->subscriber(configs.rssi_1_2());
  gz->subscriber(configs.rssi_1_3());
  gz->subscriber(configs.rssi_2_3());

  gz->publisher(configs.reset_1());
  gz->publisher(configs.reset_2());
  gz->publisher(configs.reset_3());

  /* Wait for 10 seconds, Just to finish subscribe to
   * gazebo topics */
  std::this_thread::sleep_for(std::chrono::seconds(10));  
  if (settings.flying() == true) {
    
    auto joystick_handler = [&](){
			      if (joystick_mode) {
				joystick_event_handler(joystick,
						       iris_x,
						       configs.speed(),
						       configs.just_fly());
			      }
			    };
    
    auto keyboard_handler = [&](){
			      keyboard_event_handler(iris_x,
						     configs.speed(),
						     configs.just_fly());
			    };
    
    auto joystick_events = std::async(std::launch::async, joystick_handler);
    auto keyboard_events = std::async(std::launch::async, keyboard_handler);    
    joystick_events.get();
    keyboard_events.get();
  }
}
