# include "px4_device.hh"


Px4Device::Px4Device(lt::connection_type socket,
		     lt::port_type port)
{
    
  std::string connection_url = socket + "://:" + std::to_string(port);    
  
  connect_to_quad(connection_url);
  
  discover_system();  
  
  System& system = dc_.system();  
  
  telemetry_   = std::make_shared<dronecode_sdk::Telemetry>(system);
  offboard_    = std::make_shared<dronecode_sdk::Offboard>(system);
  action_      = std::make_shared<dronecode_sdk::Action>(system);
  calibration_ = std::make_shared<dronecode_sdk::Calibration>(system);
  
  set_rate_result();
  position_ned();
  quad_health();      
  
}

ConnectionResult Px4Device::connect_to_quad(std::string connection_url)
{
  ConnectionResult connection_result;  
  connection_result = dc_.add_any_connection(connection_url);
   
  if (connection_result != ConnectionResult::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "Connection failed: "
	      << connection_result_str(connection_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return connection_result;
  }
  return connection_result;
  
}

bool Px4Device::discover_system()
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

bool Px4Device::takeoff()
{
  
  std::cout << "taking off..." << std::endl;
  const Action::Result takeoff_result = action_->takeoff();
  if(takeoff_result != Action::Result::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      << "take off failed: "
	      << Action::result_str(takeoff_result)
	      << std::endl;
    return false;
  }
  return true;
  
}

bool Px4Device::land()
{
  std::cout << "Landing..." << std::endl;
  const Action::Result land_result = action_->land();
  if (land_result != Action::Result::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "Land failed:"
	      << Action::result_str(land_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return false;
  }
  return true;

}

bool Px4Device::return_to_launch()
{
  std::cout << "return to launch position..." << std::endl;
  const Action::Result rtl_result = action_->return_to_launch();
  if (rtl_result != Action::Result::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "return to launch position failed:"
	      << Action::result_str(rtl_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return false;
  }
  return true;  
}


bool Px4Device::set_altitude_rtl_max(float meter)
{
  std::cout << "set altitude rtl..." << std::endl;
  const Action::Result rtl_altitude = action_->set_return_to_launch_return_altitude(meter);
  if (rtl_altitude != Action::Result::SUCCESS) {
    std::cout << ERROR_CONSOLE_TEXT
	      << "return to launch position failed:"
	      << Action::result_str(rtl_altitude)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return false;
  }
  return true;  
}


void Px4Device::goUp(float speed)
{
  std::cout << "To the sky !" << std::endl;
  
  offboard_->set_velocity_body({0.0f, 0.0f, -speed, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

void Px4Device::goDown(float speed)
{
  std::cout << "To the Earth !" << std::endl;

  offboard_->set_velocity_body({0.0f, 0.0f, +speed, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
  
}

void Px4Device::goRight(float speed)
{
  std::cout << "Right now !" << std::endl;
  
  offboard_->set_velocity_body({0.0f, +speed, 0.0f, 0.0f}); 
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
}

void Px4Device::goLeft(float speed)
{
  std::cout << "Left now !" << std::endl;
  
  offboard_->set_velocity_body({0.0f, -speed, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  

}

void Px4Device::init_speed()
{
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});		         
}

Offboard::Result Px4Device::start_offboard_mode()
{
  
  Offboard::Result offboard_result = offboard_->start();
    
  if (offboard_result != Offboard::Result::SUCCESS) {
    std::cerr << "Offboard::start() failed: " 
  	      << Offboard::result_str(offboard_result) << std::endl;
    
    return offboard_result; 
  }
  
  return offboard_result; 
}

void Px4Device::forward(float speed)
{
  std::cout << "go forward !" << std::endl;

  //set velocity function is going to make the quad go  all the time
  // we need to set it to zero after each keyboard touch
  
  offboard_->set_velocity_body({speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
}

void Px4Device::backward(float speed)
{
  std::cout << "go backward !" << std::endl;
    
  offboard_->set_velocity_body({-speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  


}

void Px4Device::turnToLeft(float speed)
{
  std::cout << " ... left rotate !" << std::endl;
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, -speed});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

void Px4Device::turnToRight(float speed)
{
  std::cout << " ... right rotate" << std::endl;
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, speed});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

Action::Result Px4Device::arm()
{
  Action::Result arm_result = action_->arm();
  if(arm_result != Action::Result::SUCCESS){
    std::cout << "Arming failed: "
	      << Action::result_str(arm_result)
	      << std::endl;
    return arm_result;
    
  }
  
  return arm_result;            
  
}


void Px4Device::print_position()
{
  
  telemetry_->position_async([](Telemetry::Position position){
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

Telemetry::PositionVelocityNED Px4Device::position() const
{
  // Telemetry::PositionVelocityNED po;

  // po = position_ned_.load(std::memory_order_acquire);

  //  std::lock_guard<std::mutex> lock(_position_ned_mutex);
  
  // std::cout<<"position ned: "<< _position_ned.position.north_m << std::endl;
  // std::cout<<"position ned: "<< _position_ned.position.east_m << std::endl;
  // std::cout<<"position ned: "<< _position_ned.position.down_m << std::endl;

  return _position_ned;    
}


void Px4Device::position_ned()
{
  telemetry_->position_velocity_ned_async([this](Telemetry::PositionVelocityNED pvn){
					    //std::lock_guard<std::mutex> unlock(_position_ned_mutex);
					    
					    this->_position_ned = pvn ;
					    					    
					  });   
}

Calibration::calibration_callback_t
Px4Device::create_calibration_callback(std::promise<void> &calibration_promise)
{

  return [&calibration_promise](const Calibration::Result result,
				const Calibration::ProgressData progress_data) {
	   switch (result) {
	   case Calibration::Result::SUCCESS:
	     std::cout << "--- Calibration succeeded!" << std::endl;
	     calibration_promise.set_value();
	     break;
	   case Calibration::Result::IN_PROGRESS:
	     std::cout << "    Progress: " << progress_data.progress << std::endl;
	     break;
	   case Calibration::Result::INSTRUCTION:
	     std::cout << "    Instruction: " << progress_data.status_text << std::endl;
	     break;
	   default:
	     std::cout << "--- Calibration failed with message: "
		       << Calibration::result_str(result) << std::endl;
                calibration_promise.set_value();
                break;
	   }
	 };
  
}

void Px4Device::calibrate_accelerometer()
{
  
  std::cout << "Calibrating accelerometer..." << std::endl;
  
  std::promise<void> calibration_promise;
  auto calibration_future = calibration_promise.get_future();
  
  calibration_->
    calibrate_accelerometer_async(Px4Device::create_calibration_callback(calibration_promise));
  
  calibration_future.wait();
}



void Px4Device::quad_health()
{

  while (telemetry_->health_all_ok() != true) {
    std::cout << "Vehicle is getting ready to arm" << std::endl;
    sleep_for(seconds(1));
  }
    
}


Telemetry::Result Px4Device::set_rate_result()
{
  const Telemetry::Result set_rate_result = telemetry_->set_rate_position(1.0);
  
  if (set_rate_result != Telemetry::Result::SUCCESS){
    std::cout << ERROR_CONSOLE_TEXT
	      <<"Set rate failed:"
	      << Telemetry::result_str(set_rate_result)
	      << NORMAL_CONSOLE_TEXT << std::endl;
    return set_rate_result;
  }
  return set_rate_result;
      
}
    

