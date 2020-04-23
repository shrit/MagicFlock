#include "include/px4_device.hh"

Px4Device::Px4Device(std::string socket, std::string port)
{
  std::string connection_url = socket + "://:" + port;

  connect_to_quad(connection_url);
  discover_system();
  System& system = mavsdk_.system();

  action_ = std::make_shared<mavsdk::Action>(system);
  calibration_ = std::make_shared<mavsdk::Calibration>(system);
  offboard_ = std::make_shared<mavsdk::Offboard>(system);
  shell_ = std::make_shared<mavsdk::Shell>(system);
  telemetry_ = std::make_shared<mavsdk::Telemetry>(system);

  set_rate_result();
  position_async();     // Get position updates
  landed_state_async(); // Updated at 1 HZ, which is awkward,
  flight_mode_async();  // seems to be more updated.
  quad_health();
}

ConnectionResult
Px4Device::connect_to_quad(std::string connection_url)
{
  ConnectionResult connection_result;
  connection_result = mavsdk_.add_any_connection(connection_url);

  if (connection_result != ConnectionResult::SUCCESS) {
    logger::logger_->error("Connection failed: {} ",
                           connection_result_str(connection_result));
    return connection_result;
  }
  return connection_result;
}

bool
Px4Device::discover_system()
{
  bool discovered_system = false;
  logger::logger_->debug("Waiting to discover system...");
  mavsdk_.register_on_discover([&discovered_system](uint64_t uuid) {
    logger::logger_->debug("Discovered system with UUID: {} ", uuid);
    discovered_system = true;
  });
  sleep_for(seconds(5));
  if (!discovered_system) {
    logger::logger_->error("No system found, exiting.");
  }
  return discovered_system;
}

bool
Px4Device::takeoff()
{
  const Action::Result takeoff_result = action_->takeoff();
  if (takeoff_result != Action::Result::SUCCESS) {
    logger::logger_->error("Taking off has failed: {}",
                           Action::result_str(takeoff_result));
    return false;
  }
  /*  Wait until the Flight Mode changes to takeoff */
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (landed_state() == Telemetry::LandedState::TAKING_OFF) {
      logger::logger_->debug("Landed State : {}",
                             telemetry_->landed_state_str(landed_state()));
      break;
    }
  }
  while (landed_state() == Telemetry::LandedState::TAKING_OFF) {
    logger::logger_->debug("Taking off...");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  logger::logger_->debug("Taking off has finished successfully...");
  return true;
}

bool
Px4Device::takeoff(float meters)
{
  bool altitude = set_takeoff_altitude(meters);
  if (!altitude) {
    logger::logger_->warn(
      "Set takeoff altitude has failed, Taking off with default altitude");
  }
  const Action::Result takeoff_result = action_->takeoff();
  if (takeoff_result != Action::Result::SUCCESS) {
    logger::logger_->error("Taking off has failed: {}",
                           Action::result_str(takeoff_result));
    return false;
  }
  /*  Wait until the Flight Mode changes to takeoff */
  while (true) {
    logger::logger_->debug("Landed State :{}",
                           telemetry_->landed_state_str(landed_state()));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (landed_state() == Telemetry::LandedState::TAKING_OFF) {
      logger::logger_->debug("Landed State :{}",
                             telemetry_->landed_state_str(landed_state()));
      break;
    }
  }
  while (landed_state() == Telemetry::LandedState::TAKING_OFF) {
    logger::logger_->debug("Taking off...");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  logger::logger_->debug("Taking off has finished successfully...");
  return true;
}

bool
Px4Device::land()
{
  const bool offboard = stop_offboard_mode();
  if (!offboard) {
    logger::logger_->error("Landing..., Can not stop offboard mode");
  }
  const Action::Result land_result = action_->land();

  if (land_result != Action::Result::SUCCESS) {
    logger::logger_->error("Landing command has failed: {}",
                           Action::result_str(land_result));
    return false;
  }

  while (telemetry_->in_air()) {
    logger::logger_->debug("Landing...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  logger::logger_->debug("Landed.");
  return true;
}

bool
Px4Device::return_to_launch()
{
  logger::logger_->debug("Return to launch position...");
  const Action::Result rtl_result = action_->return_to_launch();
  if (rtl_result != Action::Result::SUCCESS) {
    logger::logger_->error("Return to launch position failed: {}",
                           Action::result_str(rtl_result));
    return false;
  }
  return true;
}

bool
Px4Device::set_takeoff_altitude(float meters)
{
  logger::logger_->debug("Setting altitude takeoff to: {} ", meters, "meters");
  const Action::Result takeoff_altitude = action_->set_takeoff_altitude(meters);
  if (takeoff_altitude != Action::Result::SUCCESS) {
    logger::logger_->error("Set takeoff altitude has failed: {}",
                           Action::result_str(takeoff_altitude));
    return false;
  }
  return true;
}

bool
Px4Device::set_altitude_rtl_max(float meters)
{
  logger::logger_->debug("Set altitude rtl...");
  const Action::Result rtl_altitude =
    action_->set_return_to_launch_return_altitude(meters);
  if (rtl_altitude != Action::Result::SUCCESS) {
    logger::logger_->error("Return to launch position failed: {}",
                           Action::result_str(rtl_altitude));
    return false;
  }
  return true;
}

void
Px4Device::init_speed()
{
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

bool
Px4Device::start_offboard_mode()
{
  Offboard::Result offboard_result = offboard_->start();
  if (offboard_result != Offboard::Result::SUCCESS) {
    logger::logger_->error("Offboard::start() failed: {}",
                           Offboard::result_str(offboard_result));
    return false;
  }
  return true;
}

bool
Px4Device::stop_offboard_mode()
{
  Offboard::Result offboard_result = offboard_->stop();
  if (offboard_result != Offboard::Result::SUCCESS) {
    logger::logger_->error("Offboard::stop() failed: {}",
                           Offboard::result_str(offboard_result));
    return false;
  }
  return true;
}

/*  Use the following functions set in order to generate trajectory or a
    dataset */
void
Px4Device::up(float speed, unsigned int milliseconds_)
{
  logger::logger_->debug("Up !");
  offboard_->set_velocity_body({ 0.0f, 0.0f, -speed, 0.0f });
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::down(float speed, unsigned int milliseconds_)
{
  logger::logger_->debug("Down !");
  offboard_->set_velocity_body({ 0.0f, 0.0f, +speed, 0.0f });
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::right(float speed, unsigned int milliseconds_)
{
  logger::logger_->debug("Right now !");
  offboard_->set_velocity_body({ 0.0f, +speed, 0.0f, 0.0f });
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::left(float speed, unsigned int milliseconds_)
{
  logger::logger_->debug("Left now !");
  offboard_->set_velocity_body({ 0.0f, -speed, 0.0f, 0.0f });
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::forward(float speed, unsigned int milliseconds_)
{
  logger::logger_->debug("Forward !");
  offboard_->set_velocity_body({ speed, 0.0f, 0.0f, 0.0f });
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::backward(float speed, unsigned int milliseconds_)
{
  logger::logger_->debug("Backward !");
  offboard_->set_velocity_body({ -speed, 0.0f, 0.0f, 0.0f });
  sleep_for(milliseconds(milliseconds_));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

/*  The following function should only be used with a keyboard control
    or a joystick control. DO NOT use these functions to generate
    trajectories or a dataset of any kind. There is no guarantee that
    these functions are going to be executed at the requested number of
    times. If you would like to generate trajectory or a dataset, use the
    function with a timers that indicate the number of seconds you would
    like the quadrotors to move more precisely. Otherwise it would be
    impossible to ensure the distances between the quadrotors*/
void
Px4Device::up(float speed)
{
  logger::logger_->debug("Up !");
  offboard_->set_velocity_body({ 0.0f, 0.0f, -speed, 0.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::down(float speed)
{
  logger::logger_->debug("Down !");
  offboard_->set_velocity_body({ 0.0f, 0.0f, +speed, 0.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::right(float speed)
{
  logger::logger_->debug("Right !");
  offboard_->set_velocity_body({ 0.0f, +speed, 0.0f, 0.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::left(float speed)
{
  logger::logger_->debug("Left !");
  offboard_->set_velocity_body({ 0.0f, -speed, 0.0f, 0.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

/*  Speed in m/s */
void
Px4Device::forward(float speed)
{
  logger::logger_->debug("Forward !");
  offboard_->set_velocity_body({ speed, 0.0f, 0.0f, 0.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

/* Going forward and left at the same time in a circular movement */
void
Px4Device::forward_left(float speed)
{
  logger::logger_->debug("Forward !");
  offboard_->set_velocity_body({ speed, 0.0f, 0.0f, -30.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

/* Going forward and right at the same time in a circular movement */
void
Px4Device::forward_right(float speed)
{
  logger::logger_->debug("Forward !");
  offboard_->set_velocity_body({ speed, 0.0f, 0.0f, 30.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

void
Px4Device::backward(float speed)
{
  logger::logger_->debug("Backward !");
  offboard_->set_velocity_body({ -speed, 0.0f, 0.0f, 0.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

/* Going backward and left at the same time in a circular movement */
void
Px4Device::backward_left(float speed)
{
  logger::logger_->debug("Backward !");
  offboard_->set_velocity_body({ -speed, 0.0f, 0.0f, -30.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

/* Going backward and right at the same time in a circular movement */
void
Px4Device::backward_right(float speed)
{
  logger::logger_->debug("Backward !");
  offboard_->set_velocity_body({ -speed, 0.0f, 0.0f, 30.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

// add later the angular yaw speed
void
Px4Device::turnToLeft()
{
  logger::logger_->debug("Rotate left!");
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 270.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}
// add later the angular yaw speed
void
Px4Device::turnToRight()
{
  logger::logger_->debug("Rotate right");
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 90.0f });
  sleep_for(milliseconds(50));
  offboard_->set_velocity_body({ 0.0f, 0.0f, 0.0f, 0.0f });
}

bool
Px4Device::arm()
{
  Action::Result arm_result = action_->arm();
  if (arm_result != Action::Result::SUCCESS) {
    logger::logger_->error("Arming failed: {}", Action::result_str(arm_result));
    return false;
  }
  return true;
}

bool
Px4Device::reboot()
{
  logger::logger_->debug("Rebooting...");
  Action::Result reboot_result = action_->reboot();
  if (reboot_result != Action::Result::SUCCESS) {
    logger::logger_->error("Rebooting failed: {}",
                           Action::result_str(reboot_result));
    return false;
  }
  return true;
}

position_GPS<double>
Px4Device::get_position_GPS()
{
  position_GPS<double> pos;
  pos.latitude_deg = position_.latitude_deg;
  pos.longitude_deg = position_.longitude_deg;
  pos.absolute_altitude_m = position_.absolute_altitude_m;
  pos.relative_altitude_m = position_.relative_altitude_m;
  return pos;
}

void
Px4Device::position_async()
{
  telemetry_->position_async(
    [this](Telemetry::Position position) { this->position_ = position; });
}

void
Px4Device::landed_state_async()
{
  telemetry_->landed_state_async(
    [this](Telemetry::LandedState landed) { this->_landed_state = landed; });
}

void
Px4Device::flight_mode_async()
{
  telemetry_->flight_mode_async([this](Telemetry::FlightMode flight_mode) {
    this->_flight_mode = flight_mode;
  });
}

Telemetry::LandedState
Px4Device::landed_state() const
{
  std::lock_guard<std::mutex> lock(_landed_state_mutex);
  return _landed_state;
}

Telemetry::FlightMode
Px4Device::flight_mode() const
{
  std::lock_guard<std::mutex> lock(_flight_mode_mutex);
  return _flight_mode;
}

bool
Px4Device::execute_px4_shell_command(Shell::ShellMessage message)
{
  Shell::Result shell_results = shell_->shell_command(message);
  if (shell_results != Shell::Result::SUCCESS) {
    logger::logger_->error("Executing shell command has failed: {}",
                           shell_->result_code_str(shell_results));
    return false;
  }
  return true;
}

bool
Px4Device::receive_px4_shell_reponse()
{
  Shell::Result shell_results;
  shell_->shell_command_response_async(
    [&](Shell::Result result, Shell::ShellMessage reponse) {
      this->_shell_reponse = reponse;
      shell_results = result;
    });

  if (shell_results != Shell::Result::SUCCESS) {
    logger::logger_->error("Error reponse Px4: {}",
                           shell_->result_code_str(shell_results));
    return false;
  }
  logger::logger_->debug(_shell_reponse);
  return true;
}

Telemetry::PositionVelocityNED
Px4Device::get_position_ned() const
{
  return _position_ned;
}

double
Px4Device::DistanceFrom(std::shared_ptr<Px4Device> a)
{
  Telemetry::PositionVelocityNED a_position = a->get_position_ned();
  Telemetry::PositionVelocityNED my_position = get_position_ned();
  return CalculateDistance(my_position, a_position);
}

double
Px4Device::CalculateDistance(Telemetry::PositionVelocityNED& a,
                             Telemetry::PositionVelocityNED& b)
{
  return std::sqrt(std::pow((a.position.north_m - b.position.north_m), 2) +
                   std::pow((a.position.east_m - b.position.east_m), 2) +
                   std::pow((a.position.down_m - b.position.down_m), 2));
}

void
Px4Device::position_ned_async()
{
  telemetry_->position_velocity_ned_async(
    [this](Telemetry::PositionVelocityNED pvn) { this->_position_ned = pvn; });
}

Calibration::calibration_callback_t
Px4Device::create_calibration_callback(std::promise<void>& calibration_promise)
{

  return [&calibration_promise](const Calibration::Result result,
                                const Calibration::ProgressData progress_data) {
    switch (result) {
      case Calibration::Result::SUCCESS:
        logger::logger_->debug("--- Calibration succeeded!");
        calibration_promise.set_value();
        break;
      case Calibration::Result::IN_PROGRESS:
        logger::logger_->debug("--- Progress: {} ", progress_data.progress);
        break;
      case Calibration::Result::INSTRUCTION:
        logger::logger_->debug("--- Instruction:{}", progress_data.status_text);
        break;
      default:
        logger::logger_->debug("--- Calibration failed with message: {}",
                               Calibration::result_str(result));
        calibration_promise.set_value();
        break;
    }
  };
}

void
Px4Device::calibrate_accelerometer()
{
  logger::logger_->debug("Calibrating accelerometer...");
  std::promise<void> calibration_promise;
  auto calibration_future = calibration_promise.get_future();

  calibration_->calibrate_accelerometer_async(
    Px4Device::create_calibration_callback(calibration_promise));
  calibration_future.wait();
}

void
Px4Device::quad_health()
{
  telemetry_->health_async(
    [this](Telemetry::Health health) { this->health_ = health; });

  if (health_.gyrometer_calibration_ok == false) {
    logger::logger_->error("Gyrometer is not calibrated please calibrate the "
                           "gyrometer and try later ");
  } else if (health_.accelerometer_calibration_ok == false) {
    logger::logger_->error("Accelerometer is not calibrated please calibrate "
                           "the accelerometer and try later");
  } else if (health_.magnetometer_calibration_ok == false) {
    logger::logger_->error("Magnetometer is not calibrated please calibrate "
                           "the magnetometer and try later");
  } else if (health_.level_calibration_ok == false) {
    logger::logger_->error("Please check the caibration level of the vehicle");
  } else if (health_.local_position_ok == false) {
    logger::logger_->error("NO local position, can not fly in position mode");
  } else if (health_.global_position_ok == false) {
    logger::logger_->error("No GPS position, can not fly in position mode");
  } else if (health_.home_position_ok == false) {
    logger::logger_->error("Home position is not initialized properly");
  }
}

Telemetry::Result
Px4Device::set_rate_result()
{
  const Telemetry::Result set_rate_result = telemetry_->set_rate_position(1.0);
  if (set_rate_result != Telemetry::Result::SUCCESS) {
    logger::logger_->error("Set rate failed:",
                           Telemetry::result_str(set_rate_result));
    return set_rate_result;
  }
  return set_rate_result;
}

// Handles Action result
inline void
Px4Device::action_error_exit(Action::Result result, const std::string& msg)
{
  if (result != Action::Result::SUCCESS) {
    logger::logger_->error(msg, Action::result_str(result));
    exit(EXIT_FAILURE);
  }
}

// Handles Offboard result
inline void
Px4Device::offboard_error_exit(Offboard::Result result, const std::string& msg)
{
  if (result != Offboard::Result::SUCCESS) {
    std::cerr << ERROR_CONSOLE_TEXT << msg << Offboard::result_str(result)
              << NORMAL_CONSOLE_TEXT;
    exit(EXIT_FAILURE);
  }
}

// Handles Connection result
inline void
Px4Device::connection_error_exit(ConnectionResult result,
                                 const std::string& msg)
{
  if (result != ConnectionResult::SUCCESS) {
    std::cerr << ERROR_CONSOLE_TEXT << msg << connection_result_str(result)
              << NORMAL_CONSOLE_TEXT;
    exit(EXIT_FAILURE);
  }
}
