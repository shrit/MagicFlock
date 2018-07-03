



# include "controller.hh"
# include <Qt>

Controller::Controller(QObject* parent)
  :  QObject(parent), m_gamepad(0)
{







  this->connect(dc, "udp://:11454");
  this->discover_system(dc);
  this->get_position(telemetry);
  this->quad_health(telemetry);

  this->


  
  auto gamepads = QGamepadManager::instance()->connectedGamepads();
  
  
  if (gamepads.isEmpty()) {
    std::cerr << "Did not find any connected gamepads" << std::endl;
    return;
  }
  
  m_gamepad = new QGamepad(*gamepads.begin(), this);
  
  connect(m_gamepad, &QGamepad::axisLeftXChanged,
	  this, [](double value){
		  std::cout << "Left X" << value <<std::endl;
		  
		});
  connect(m_gamepad, &QGamepad::axisLeftYChanged,
	  this, [](double value){
		  std::cout << "Left Y" << value << std ::endl;
    });
    connect(m_gamepad, &QGamepad::axisRightXChanged,
	    this, [](double value){
		    std::cout << "Right X" << value << std::endl;
		    
		  });
    connect(m_gamepad, &QGamepad::axisRightYChanged,
	    this, [](double value){
		    std::cout << "Right Y" << value << std::endl;
		    
		  });
    
    connect(m_gamepad, &QGamepad::buttonAChanged,
	    this, [](bool pressed){
		    std::cout << "Button A" << pressed << std::endl;
		  });
    
    connect(m_gamepad, &QGamepad::buttonBChanged,
	    this, [](bool pressed){
		    std::cout << "Button B" << pressed << std::end;
		    
		  });
    
    connect(m_gamepad, &QGamepad::buttonXChanged,
	    this, [](bool pressed){
		    std::cout << "Button X" << pressed << std::endl;
		    
		  });
    connect(m_gamepad, &QGamepad::buttonYChanged,
	    this, [](bool pressed){
		    std::cout << "Button Y" << pressed << std::endl;
		    
		  });
    
    connect(m_gamepad, &QGamepad::buttonL1Changed,
	    this, [](bool pressed){
		    std::cout << "Button L1" << pressed << std::endl;
		    
		  });
    
    connect(m_gamepad, &QGamepad::buttonR1Changed,
	    this, [](bool pressed){
		    std::cout << "Button R1" << pressed << std::endl;
		    this->takeoff(action)
		  });
    
    connect(m_gamepad, &QGamepad::buttonL2Changed,
	    this, [](double value){
		    std::cout  << "Button L2: " << value << std::endl;
		    
		  });
    
    connect(m_gamepad, &QGamepad::buttonR2Changed,
	    this, [](double value){
		    std::cout  << "Button R2: " << value << std::endl;
		    this->land(action)
		  });
    
    connect(m_gamepad, &QGamepad::buttonSelectChanged,
	    this, [](bool pressed){
		    std::cout  << "Button Select" << pressed << std::endl;
		    
		  });
    
    connect(m_gamepad, &QGamepad::buttonStartChanged,
	    this, [](bool pressed){
		    std::cout  << "Button Start" << pressed << std::endl;
		    
		  });
    connect(m_gamepad, &QGamepad::buttonGuideChanged,
	    this, [](bool pressed){
		    std::cout  << "Button Guide" << pressed << std::endl;
		    
		  });
  
}


ConnectionResult Controller::connect(DroneCore dc,
				     std::string connection_url)
{
  //  connection_url = "udp://:14540";  
  ConnectionResult connection_result;  
  connection_result = dc.add_any_connection(connection_url);
  
  if (connection_result != ConnectionResult::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "Connection failed: "
	      << connection_result_str(connection_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return connection_result;
  }
  return connection_result;
  
}

System& Controller::discover_system(Dronecore dc)
{

  bool discovered_system = false;
    
  std::cout << "Waiting to discover system..." << std::endl;
  dc.register_on_discover([&discovered_system](uint64_t uuid) {
			    std::cout << "Discovered system with UUID: "
				      << uuid << std::endl;
			    discovered_system = true;
			  });

  sleep_for(seconds(2));

  if (!discovered_system) {
    std::cout << ERROR_CONSOLE_TEXT
	      <<"No system found, exiting." << NORMAL_CONSOLE_TEXT
	      << std::endl;
    return false;
    }

  System& system = dc.system();
  
  return system;

}

bool Controller::takeoff(std::shared_ptr<dronecore::Action> action)
{
  
  std::cout << "taking off" << std::endl;
  const ActionResult takeoff_result = action->takeoff();
  if(takeoff_result != ActionResult::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      << "take off failed: "
	      << action_result_str(takeoff_result)
	      << std::endl;
    return false;
  }
  return true;
  
}

bool Controller::land(std::shared_ptr<dronecore::Action> action)
{
  std::cout << "Landing..." << std::endl;
  const ActionResult land_result = action->land();
  if (land_result != ActionResult::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "Land failed:"
	      << action_result_str(land_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return false;
  }
  return true;

}


void Controller::goUp(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "To the sky !" << std::endl;
  
  offboard->set_velocity_body({0.0f, 0.0f, -3.0f, 0.0f});
  sleep_for(seconds(2));
  return true;
      
    
}

bool Controller::goDown(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "To the Earth !" << std::endl;
  offboard->set_velocity_body({0.0f, 0.0f, +3.0f, 0.0f});
  sleep_for(seconds(2));
  return true;
  
}

bool Controller::goRight(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "Right now !" << std::endl;
  
  offboard->set_velocity_body({0.0f, 2.0f, 0.0f, 0.0f});
  sleep_for(seconds(2));
  return true;
  
}

bool Controller::goLeft(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "Left now !" << std::endl;
  
  offboard->set_velocity_body({0.0f, -2.0f, 0.0f, 0.0f});
  sleep_for(seconds(2));
  return true;

}

bool Controller::forward(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "go forward !" << std::endl;  
  offboard->set_velocity_body({2.0f, 0.0f, 0.0f, 0.0f});
  sleep_for(seconds(2));
  return true;

}

bool Controller::backward(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "go backward !" << std::endl;
    
  offboard->set_velocity_body({-2.0f, 0.0f, 0.0f, 0.0f});
  sleep_for(seconds(2));
  return true;


}

bool Controller::turnToLeft(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << " ... Left rotate !" << std::endl;
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 10.0f});
  sleep_for(seconds(1));
    return true;
}

bool Controller::turnToRight(std::shared_ptr<dronecore::Offboard> offboard)
{
    std::cout << " ... right rotate" << std::endl;
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, -10.0f});
    sleep_for(seconds(1));
    return true;
}


ActionResult Controller::arm(std::shared_ptr<dronecore::Action> action)
{
  ActionResult arm_result = action->arm();
  if(arm_result != ActionResult::SUCCESS){
    std::cout << "Arming failed: "
	      << action_result_str(arm_result)
	      << std::endl;
    return arm_result;
    
  }
  
  return arm_result;            
  
}


void Controller::get_position(std::shared_ptr<dronecore::Telemetry> telemetry)
{
  
  telemetry->position_async([](Telemetry::Position position){
			      std::cout << TELEMETRY_CONSOLE_TEXT
 					<< "Altitude : "
					<< position.relative_altitude_m
					<< " m"
					<< "Latitude"
					<< position.latitude_deg << " deg"
					<< "Longtitude"
					<< position.longitude_deg <<" deg"
					<< std::endl;

			    });
  
}



void Controller::quad_health(std::shared_ptr<dronecore::Telemetry> telemetry)
{

  while (telemetry->health_all_ok() != true) {
    std::cout << "Vehicle is getting ready to arm" << std::endl;
    std::this_thread::sleep_for(seconds(1));
  }
  
  
}


dronecore::Telemetry::Result Controller::set_rate_result(							 std::shared_ptr<dronecore::Telemetry> telemetry)
{
  const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
  
  if (set_rate_result != Telemetry::Result::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      <<"Set rate failed:"
	      << Telemetry::result_str(set_rate_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return set_rate_result;
  }
  
      
}
  
  
