# include "px4_device.hh"

Px4Device::Px4Device(lt::connection_type socket,
		     lt::port_type port)
{
  std::string connection_url = socket + "://:" + std::to_string(port);

  connect_to_quad(connection_url);
  discover_system();
  System& system = mavsdk_.system();

  telemetry_   = std::make_shared<mavsdk::Telemetry>(system);
  offboard_    = std::make_shared<mavsdk::Offboard>(system);
  action_      = std::make_shared<mavsdk::Action>(system);
  calibration_ = std::make_shared<mavsdk::Calibration>(system);

  set_rate_result();
  position_async(); // Get position updates
  landed_state_async(); // Updated at 1 HZ, which is awkward,
  flight_mode_async(); // seems to be more updated.
  quad_health();  //Stop testing quad health for some moments
}

ConnectionResult Px4Device::connect_to_quad(std::string connection_url)
{
  ConnectionResult connection_result;
  connection_result = mavsdk_.add_any_connection(connection_url);

  if (connection_result != ConnectionResult::SUCCESS) {
    LogErr() << ERROR_CONSOLE_TEXT
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
  mavsdk_.register_on_discover([&discovered_system](uint64_t uuid) {
				 LogInfo() << "Discovered system with UUID: "
					   << uuid;
				 discovered_system = true;
			       });
  sleep_for(seconds(5));

  if (!discovered_system) {
    LogErr() << ERROR_CONSOLE_TEXT
	     <<"No system found, exiting."
	     << NORMAL_CONSOLE_TEXT;
  }
  return discovered_system;
}

bool Px4Device::takeoff()
{
  const Action::Result takeoff_result = action_->takeoff();
  if(takeoff_result != Action::Result::SUCCESS){
    LogErr() << ERROR_CONSOLE_TEXT
	     << "Taking off has failed: "
	     << Action::result_str(takeoff_result);
    return false;
  }
    /*  Wait until the Flight Mode changes to takeoff */
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (flight_mode() == Telemetry::FlightMode::TAKEOFF) {
      LogInfo() << "Flight Mode : "<< telemetry_->flight_mode_str(flight_mode());
      break;
    }
  }  
  while (flight_mode() == Telemetry::FlightMode::TAKEOFF) {   
    LogInfo() << "Taking off..." ;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  LogInfo() << "Taking off has finished successfully...";

  return true;
}

bool Px4Device::takeoff(float meters)
{
  bool altitude = set_takeoff_altitude(meters);
  if (!altitude) {
    LogWarn() << "Set takeoff altitude has failed, Taking off with default altitude";
  }
  const Action::Result takeoff_result = action_->takeoff();
  if (takeoff_result != Action::Result::SUCCESS) {
    LogErr() << ERROR_CONSOLE_TEXT
	     << "Taking off has failed: "
	     << Action::result_str(takeoff_result);
    return false;
  }
    /*  Wait until the Flight Mode changes to takeoff */
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (flight_mode() == Telemetry::FlightMode::TAKEOFF) {
      LogInfo() << "Flight Mode : "<< telemetry_->flight_mode_str(flight_mode());
      break;
    }
  }  
  while (flight_mode() == Telemetry::FlightMode::TAKEOFF) {   
    LogInfo() << "Taking off..." ;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  LogInfo() << "Taking off has finished successfully...";
  return true;
}

bool Px4Device::land()
{  
  const Action::Result land_result = action_->land();
  
  if (land_result != Action::Result::SUCCESS) {
    LogErr() << ERROR_CONSOLE_TEXT
	     << "Landing command has failed:"
	     << Action::result_str(land_result)
	     << NORMAL_CONSOLE_TEXT ;
    return false;
  }

  while (telemetry_->in_air()) {
    LogInfo() << "Landing..." ;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }  
  LogInfo() << "Landed." ;
  return true;
}

bool Px4Device::return_to_launch()
{
  LogInfo() << "return to launch position..." ;
  const Action::Result rtl_result = action_->return_to_launch();
  if (rtl_result != Action::Result::SUCCESS) {
    LogErr() << ERROR_CONSOLE_TEXT
	     << "return to launch position failed:"
	     << Action::result_str(rtl_result)
	     << NORMAL_CONSOLE_TEXT ;
    return false;
  }
  return true;
}

bool Px4Device::set_takeoff_altitude(float meters)
{
  LogInfo() << "Setting altitude takeoff to "<< meters << "meters";
  const Action::Result takeoff_altitude =
    action_->set_takeoff_altitude(meters);
  if (takeoff_altitude != Action::Result::SUCCESS) {
    LogErr() << ERROR_CONSOLE_TEXT
	     << "Set takeoff altitude has failed:"
	     << Action::result_str(takeoff_altitude)
	     << NORMAL_CONSOLE_TEXT ;
    return false;
  }
  return true;
}

bool Px4Device::set_altitude_rtl_max(float meters)
{
  LogInfo() << "set altitude rtl..." ;
  const Action::Result rtl_altitude =
    action_->set_return_to_launch_return_altitude(meters);
  if (rtl_altitude != Action::Result::SUCCESS) {
    LogErr() << ERROR_CONSOLE_TEXT
	     << "return to launch position failed:"
	     << Action::result_str(rtl_altitude)
	     << NORMAL_CONSOLE_TEXT ;
    return false;
  }
  return true;
}

void Px4Device::init_speed()
{
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

bool Px4Device::start_offboard_mode()
{

  Offboard::Result offboard_result = offboard_->start();

  if (offboard_result != Offboard::Result::SUCCESS) {
    std::cerr << "Offboard::start() failed: "
  	      << Offboard::result_str(offboard_result) ;

    return false;
  }
  return true;
}

/*  Use the following functions set in order to generate trajectory
    or a dataset */

void Px4Device::up(float speed, unsigned int milliseconds_)
{
  LogInfo() << "Up !";

  offboard_->set_velocity_body({0.0f, 0.0f, -speed, 0.0f});
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

void Px4Device::down(float speed, unsigned int milliseconds_)
{
  LogInfo() << "Down !";

  offboard_->set_velocity_body({0.0f, 0.0f, +speed, 0.0f});
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

void Px4Device::right(float speed, unsigned int milliseconds_)
{
  LogInfo() << "Right now !";

  offboard_->set_velocity_body({0.0f, +speed, 0.0f, 0.0f});
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

void Px4Device::left(float speed, unsigned int milliseconds_)
{
  LogInfo() << "Left now !";

  offboard_->set_velocity_body({0.0f, -speed, 0.0f, 0.0f});
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

void Px4Device::forward(float speed, unsigned int milliseconds_)
{
  LogInfo() << " forward !";
  
  offboard_->set_velocity_body({speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

void Px4Device::backward(float speed, unsigned int milliseconds_)
{
  LogInfo() << " backward !";

  offboard_->set_velocity_body({-speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

/*  The following function should only be used with a keyboard control
 or a joystick control. DO NOT use these function to generate
 trajectories or a dataset of any kind. There is no garantie that
 these function are going to be executed at the requested number of
 times. If you would like to generate trajectory or a dataset, use the
 function with a timers that indicate the number of seconds you would
 like the quadcopter to move more precisely. Otherwise it would be
 impossible to ensure the distances between the quadrotors*/

void Px4Device::up(float speed)
{
  LogInfo() << "Up !";

  offboard_->set_velocity_body({0.0f, 0.0f, -speed, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

void Px4Device::down(float speed)
{
  LogInfo() << "Down !";

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

/*  Speed in m/s */
void Px4Device::forward(float speed)
{
  LogInfo() << " forward !" ;
  // we need to set it to zero after each keyboard touch
  offboard_->set_velocity_body({speed, 0.0f, 0.0f, 0.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

/* Going forward and left at the same time in a circular movement */
void Px4Device::forward_left(float speed)
{
  LogInfo() << " forward !" ;
  // we need to set it to zero after each keyboard touch
  offboard_->set_velocity_body({speed, 0.0f, 0.0f, -30.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

/* Going forward and right at the same time in a circular movement */
void Px4Device::forward_right(float speed)
{
  LogInfo() << " forward !" ;
  // we need to set it to zero after each keyboard touch
  offboard_->set_velocity_body({speed, 0.0f, 0.0f, 30.0f});
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

/* Going backward and left at the same time in a circular movement */
void Px4Device::backward_left(float speed)
{
  LogInfo() << " backward !" ;

  offboard_->set_velocity_body({-speed, 0.0f, 0.0f, -30.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

/* Going backward and right at the same time in a circular movement */
void Px4Device::backward_right(float speed)
{
  LogInfo() << " backward !" ;

  offboard_->set_velocity_body({-speed, 0.0f, 0.0f, 30.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

// add later the angular yaw speed
void Px4Device::turnToLeft()
{
  LogInfo() << " ... left rotate !" ;
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 270.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}
// add later the angular yaw speed
void Px4Device::turnToRight()
{
  LogInfo() << " ... right rotate" ;
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 90.0f});
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
}

bool Px4Device::arm()
{
  Action::Result arm_result = action_->arm();
  if(arm_result != Action::Result::SUCCESS){
    LogInfo() << "Arming failed: "
	      << Action::result_str(arm_result);
    return false;
  }
  return true;
}

bool Px4Device::reboot()
{
  LogInfo() << "Rebooting..." ;
  Action::Result reboot_result = action_->reboot();
  if(reboot_result != Action::Result::SUCCESS){
    LogInfo() << "Rebooting failed: "
	      << Action::result_str(reboot_result) ;
    return false;
  }
  return true;
}

lt::position_GPS<double> Px4Device::get_position_GPS()
{
  lt::position_GPS<double> pos;
  pos.latitude_deg = position_.latitude_deg;
  pos.longitude_deg = position_.longitude_deg;
  pos.absolute_altitude_m = position_.absolute_altitude_m;
  pos.relative_altitude_m = position_.relative_altitude_m;  
  return pos;
}

void Px4Device::position_async()
{
  telemetry_->position_async([this](Telemetry::Position position){
			       this->position_ = position;
			     });
}			    

void Px4Device::landed_state_async()
{
  telemetry_->landed_state_async([this](Telemetry::LandedState landed){
			       this->_landed_state = landed;
			     });
}

void Px4Device::flight_mode_async()
{
  telemetry_->flight_mode_async([this](Telemetry::FlightMode flight_mode){
			       this->_flight_mode = flight_mode;
			     });
}

Telemetry::LandedState Px4Device::landed_state() const
{
  std::lock_guard<std::mutex> lock(_landed_state_mutex);
  return _landed_state; 
}

Telemetry::FlightMode Px4Device::flight_mode() const
{
  std::lock_guard<std::mutex> lock(_flight_mode_mutex);
  return _flight_mode; 
}

Telemetry::PositionVelocityNED Px4Device::get_position_ned() const
{  return _position_ned; }

double Px4Device::DistanceFrom(std::shared_ptr<Px4Device> a)
{
  Telemetry::PositionVelocityNED a_position = a->get_position_ned();
  Telemetry::PositionVelocityNED my_position = get_position_ned();
  return CalculateDistance (my_position, a_position);
}

double Px4Device::CalculateDistance (Telemetry::PositionVelocityNED& a,
				     Telemetry::PositionVelocityNED& b)
{
  return std::sqrt( std::pow ((a.position.north_m - b.position.north_m), 2) +
		    std::pow ((a.position.east_m - b.position.east_m), 2) +
		    std::pow ((a.position.down_m - b.position.down_m), 2)
		    );
}

void Px4Device::position_ned_async()
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
  telemetry_->health_async([this](Telemetry::Health health){
			     this->health_ = health;
			   });
  
  if (health_.gyrometer_calibration_ok == false) {
    LogErr() << "Gyrometer is not calibrated please calibrate the gyrometer and try later ";
  } else if (health_.accelerometer_calibration_ok == false) {
    LogErr() << "Accelerometer is not calibrated please calibrate the accelerometer and try later";
  } else if (health_.magnetometer_calibration_ok == false) {
    LogErr() << "Magnetometer is not calibrated please calibrate the magnetometer and try later";
  } else if (health_.level_calibration_ok == false) {
    LogErr() << "Please check the caibration level of the vehicle";
  } else if (health_.local_position_ok == false) {
    LogErr() << "NO local position, can not fly in position mode";
  } else if (health_.global_position_ok == false) {
    LogErr() << "No GPS position, can not fly in position mode";
  } else if (health_.home_position_ok == false) {
    LogErr() << "Home position is not initialized properly";
  }
}

Telemetry::Result Px4Device::set_rate_result()
{
  const Telemetry::Result set_rate_result = telemetry_->set_rate_position(1.0);

  if (set_rate_result != Telemetry::Result::SUCCESS){
    LogErr() << ERROR_CONSOLE_TEXT
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
