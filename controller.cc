# include "controller.hh"

Controller::Controller()
{
       
}


ConnectionResult Controller::connect_to_quad(DroneCore& dc,
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

bool Controller::discover_system(DroneCore& dc)
{

  bool discovered_system = false;
    
  std::cout << "Waiting to discover system..." << std::endl;
  dc_.register_on_discover([&discovered_system](uint64_t uuid) {
			    std::cout << "Discovered system with UUID: "
				      << uuid << std::endl;
			    discovered_system = true;
			  });

  sleep_for(seconds(5));

  if (!discovered_system) {
    std::cout << ERROR_CONSOLE_TEXT
	      <<"No system found, exiting." << NORMAL_CONSOLE_TEXT
	      << std::endl;
  
  }

  return discovered_system;


}

bool Controller::takeoff(std::shared_ptr<dronecore::Action> action)
{
  
  std::cout << "taking off..." << std::endl;
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
  
  offboard->set_velocity_body({0.0f, 0.0f, -10.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

void Controller::goDown(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "To the Earth !" << std::endl;

  offboard->set_velocity_body({0.0f, 0.0f, +3.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
  
}

void Controller::goRight(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "Right now !" << std::endl;
  
  offboard->set_velocity_body({0.0f, 10.0f, 0.0f, 0.0f}); 
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
}

void Controller::goLeft(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "Left now !" << std::endl;
  
  offboard->set_velocity_body({0.0f, -10.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  

}


void Controller::init_speed(std::shared_ptr<dronecore::Offboard> offboard)
{
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});		         
}


Offboard::Result Controller::start_offboard_mode(std::shared_ptr<dronecore::Offboard> offboard)
{
  
  Offboard::Result offboard_result = offboard->start();
  
  /*
   * Async code to start offboard mode in async, 
   * need to work on it
   */
  
  //Offboard::Result offboard_result;
  
  // offboard->start_async([&offboard_result](){
  
  // 			  if (offboard_result != Offboard::Result::SUCCESS) {
  // 			    std::cerr << "Offboard::start() failed: " 
  // 				      << Offboard::result_str(offboard_result) << std::endl;
  
  // 			    return offboard_result;
  // 			  }
			  			  			  
  // 			});
  
  if (offboard_result != Offboard::Result::SUCCESS) {
    std::cerr << "Offboard::start() failed: " 
  	      << Offboard::result_str(offboard_result) << std::endl;
    
    return offboard_result; 
  }
  
  return offboard_result; 
}

void Controller::forward(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "go forward !" << std::endl;

  //set velocity function is going to make the quad go  all the time
  // we need to set it to zero after each keyboard touch
  
  offboard->set_velocity_body({10.0f, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
}

void Controller::backward(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << "go backward !" << std::endl;
    
  offboard->set_velocity_body({-10.0f, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  


}

void Controller::turnToLeft(std::shared_ptr<dronecore::Offboard> offboard)
{
  std::cout << " ... left rotate !" << std::endl;
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, -10.0f});
  sleep_for(milliseconds(50));
  offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

void Controller::turnToRight(std::shared_ptr<dronecore::Offboard> offboard)
{
    std::cout << " ... right rotate" << std::endl;
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 10.0f});
    sleep_for(milliseconds(50));
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
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
    sleep_for(seconds(1));
  }
  
  
}


Telemetry::Result Controller::set_rate_result(std::shared_ptr<dronecore::Telemetry> telemetry)
{
  const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
  
  if (set_rate_result != Telemetry::Result::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      <<"Set rate failed:"
	      << Telemetry::result_str(set_rate_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return set_rate_result;
  }
  return set_rate_result;
      
}
    

