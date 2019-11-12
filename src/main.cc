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
# include "algo/training.hh"
# include "algo/generate_data_set.hh"
# include "algo/quadrotor.hh"
# include "algo/supervised_learning.hh"
# include "algo/trajectory_noise.hh"
# include "config_ini.hh"
# include "data_set.hh"
# include "gazebo.hh"
# include "global.hh"
# include "../third_party/joystick/joystick.hh"
# include "keyboard.hh"
# include "log.hh"
# include "px4_device.hh"
# include "settings.hh"

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

/*
 *  Main file: Start one controller by quadcopters
 */
int main(int argc, char* argv[])
{
  if (argc < 2) {
    LogErr() << "Please specify if you want to train or test a controller!!" ;
    exit(0);
  }
  
  Settings settings(argc, argv);
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

  if (settings.training() == true) {
    LogInfo() << "Start training...";
    Train trainer;    
    trainer.load_data_set(settings.dataset());
    trainer.run(settings);
    LogInfo() << "Finished training...";
    exit(0);
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

      
  ////////////
  // Gazebo //
  ///////////

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

  /*  Create a vector of quadrotors, each one has an id + a name  */
  /*  Try to see if it is possible or efficient to merge quadrotors +
      device controller */
  std::vector<Quadrotor<Gazebo>> quadrotors;
  quadrotors.emplace_back(0, "leader");
  quadrotors.emplace_back(1, "follower_1");
  quadrotors.emplace_back(2, "follower_2");
  
  /*  1: Generate a dataset
   *  2: Train the model on the dataset
   *  3: Test the trained model
   */

  if (settings.generate() == true) {
    Generator<Px4Device, Gazebo> generator(iris_x, quadrotors, gz);
    generator.run(settings);
    
  } else if (settings.testing() == true) {
    Supervised_learning<Px4Device, Gazebo> slearning(iris_x, quadrotors, gz);
    slearning.run(settings);
    
  } else if (settings.trajectory() == true) {
    TrajectoryNoise<Px4Device, Gazebo> trajectory_noise(iris_x, gz);
    trajectory_noise.run();

  } else if (settings.flying() == true) {
        
    auto joystick_handler = [&](){
			      if (joystick_mode) {
				joystick_event_handler(joystick,
						       iris_x,
						       configs.speed(),
						       configs.just_fly());			}
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
