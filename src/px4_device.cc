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
  //  quad_health();      
  
}

ConnectionResult Px4Device::connect_to_quad(std::string connection_url)
{
  ConnectionResult connection_result;  
  connection_result = dc_.add_any_connection(connection_url);
   
  if (connection_result != ConnectionResult::SUCCESS) {
    LogInfo() << ERROR_CONSOLE_TEXT
	      << "Connection failed: "
	      << connection_result_str(connection_result)
	      << NORMAL_CONSOLE_TEXT ;
    return connection_result;
  }
  return connection_result;
  
}

bool Px4Device::discover_system()
{

  bool discovered_system = false;
    
  LogInfo() << "Waiting to discover system..." ;
  dc_.register_on_discover([&discovered_system](uint64_t uuid) {
			     LogInfo() << "Discovered system with UUID: "
				       << uuid ;
			     discovered_system = true;
			   });

  sleep_for(seconds(5));

  if (!discovered_system) {
    LogInfo() << ERROR_CONSOLE_TEXT
	      <<"No system found, exiting." << NORMAL_CONSOLE_TEXT
	      ;
  
  }

  return discovered_system;
}

bool Px4Device::takeoff()
{
  
  LogInfo() << "taking off..." ;
  const Action::Result takeoff_result = action_->takeoff();
  if(takeoff_result != Action::Result::SUCCESS){
    LogInfo() << ERROR_CONSOLE_TEXT
	      << "take off failed: "
	      << Action::result_str(takeoff_result)
	      ;
    return false;
  }
  return true;
  
}

bool Px4Device::land()
{
  LogInfo() << "Landing..." ;
  const Action::Result land_result = action_->land();
  if (land_result != Action::Result::SUCCESS) {
    LogInfo() << ERROR_CONSOLE_TEXT
	      << "Land failed:"
	      << Action::result_str(land_result)
	      << NORMAL_CONSOLE_TEXT ;
    return false;
  }
  return true;

}

bool Px4Device::return_to_launch()
{
  LogInfo() << "return to launch position..." ;
  const Action::Result rtl_result = action_->return_to_launch();
  if (rtl_result != Action::Result::SUCCESS) {
    LogInfo() << ERROR_CONSOLE_TEXT
	      << "return to launch position failed:"
	      << Action::result_str(rtl_result)
	      << NORMAL_CONSOLE_TEXT ;
    return false;
  }
  return true;  
}


bool Px4Device::set_altitude_rtl_max(float meter)
{
  LogInfo() << "set altitude rtl..." ;
  const Action::Result rtl_altitude = action_->set_return_to_launch_return_altitude(meter);
  if (rtl_altitude != Action::Result::SUCCESS) {
    LogInfo() << ERROR_CONSOLE_TEXT
	      << "return to launch position failed:"
	      << Action::result_str(rtl_altitude)
	      << NORMAL_CONSOLE_TEXT ;
    return false;
  }
  return true;  
}


void Px4Device::up(float speed)
{
  LogInfo() << "To the sky !" ;
  
  offboard_->set_velocity_body({0.0f, 0.0f, -speed, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

void Px4Device::down(float speed)
{
  LogInfo() << "To the Earth !" ;

  offboard_->set_velocity_body({0.0f, 0.0f, +speed, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
  
}

void Px4Device::right(float speed)
{
  LogInfo() << "Right now !" ;
  
  offboard_->set_velocity_body({0.0f, +speed, 0.0f, 0.0f}); 
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
}

void Px4Device::left(float speed)
{
  LogInfo() << "Left now !" ;
  
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
  	      << Offboard::result_str(offboard_result) ;
    
    return offboard_result; 
  }
  
  return offboard_result; 
}

void Px4Device::forward(float speed)
{
  LogInfo() << " forward !" ;

  //set velocity function is ing to make the quad   all the time
  // we need to set it to zero after each keyboard touch
  
  offboard_->set_velocity_body({speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
}

void Px4Device::backward(float speed)
{
  LogInfo() << " backward !" ;
    
  offboard_->set_velocity_body({-speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  


}

void Px4Device::turnToLeft(float speed)
{
  LogInfo() << " ... left rotate !" ;
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, -speed});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

void Px4Device::turnToRight(float speed)
{
  LogInfo() << " ... right rotate" ;
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, speed});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});  
    
}

Action::Result Px4Device::arm()
{
  Action::Result arm_result = action_->arm();
  if(arm_result != Action::Result::SUCCESS){
    LogInfo() << "Arming failed: "
	      << Action::result_str(arm_result)
	      ;
    return arm_result;
    
  }
  
  return arm_result;            
  
}

Action::Result Px4Device::reboot()
{

  LogInfo() << "Rebooting..." ;
  Action::Result reboot_result = action_->reboot();
  if(reboot_result != Action::Result::SUCCESS){
    LogInfo() << "Rebooting failed: "
	      << Action::result_str(reboot_result)
	      ;
    return reboot_result;
    
  }
  
  return reboot_result;            
  
}

void Px4Device::print_position()
{
  
  telemetry_->position_async([](Telemetry::Position position){
			       LogInfo() << TELEMETRY_CONSOLE_TEXT
					 << "Altitude : "
					 << position.relative_altitude_m
					 << " m"
					 << "Latitude"
					 << position.latitude_deg << " deg"
					 << "Longtitude"
					 << position.longitude_deg <<" deg"
					 ;

			     });
  
}

Telemetry::PositionVelocityNED Px4Device::position() const
{  
  return _position_ned;    
}

void Px4Device::position_ned()
{
  telemetry_->position_velocity_ned_async([this](Telemetry::PositionVelocityNED pvn){
					    
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
	     LogInfo() << "--- Calibration succeeded!" ;
	     calibration_promise.set_value();
	     break;
	   case Calibration::Result::IN_PROGRESS:
	     LogInfo() << "    Progress: " << progress_data.progress ;
	     break;
	   case Calibration::Result::INSTRUCTION:
	     LogInfo() << "    Instruction: " << progress_data.status_text ;
	     break;
	   default:
	     LogInfo() << "--- Calibration failed with message: "
		       << Calibration::result_str(result) ;
                calibration_promise.set_value();
                break;
	   }
	 };
  
}

void Px4Device::calibrate_accelerometer()
{
  
  LogInfo() << "Calibrating accelerometer..." ;
  
  std::promise<void> calibration_promise;
  auto calibration_future = calibration_promise.get_future();
  
  calibration_->
    calibrate_accelerometer_async(Px4Device::create_calibration_callback(calibration_promise));
  
  calibration_future.wait();
}



void Px4Device::quad_health()
{

  while (telemetry_->health_all_ok() != true) {
    LogInfo() << "Vehicle is getting ready to arm" ;
    sleep_for(seconds(1));
  }
    
}


Telemetry::Result Px4Device::set_rate_result()
{
  const Telemetry::Result set_rate_result = telemetry_->set_rate_position(1.0);
  
  if (set_rate_result != Telemetry::Result::SUCCESS){
    LogInfo() << ERROR_CONSOLE_TEXT
	      <<"Set rate failed:"
	      << Telemetry::result_str(set_rate_result)
	      << NORMAL_CONSOLE_TEXT ;
    return set_rate_result;
  }
  return set_rate_result;
      
}

// Handles Action result
inline void Px4Device::action_error_exit(Action::Result result, const std::string &msg)
{
  if (result != Action::Result::SUCCESS) {
    std::cerr << ERROR_CONSOLE_TEXT << msg << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT ;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard result
inline void Px4Device::offboard_error_exit(Offboard::Result result, const std::string &msg)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << msg << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT ;
        exit(EXIT_FAILURE);
    }
}

// Handles Connection result
inline void Px4Device::connection_error_exit(ConnectionResult result, const std::string &msg)
{
  if (result != ConnectionResult::SUCCESS) {
        std::cerr << ERROR_CONSOLE_TEXT << msg << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT ;
        exit(EXIT_FAILURE);
    }
}
