/**
 * @file main.cc
 * @brief allow user to control the quadrotor using joystick
 * and keyboard
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 *
 */

/*  C++ Standard library include */
#include <chrono>
#include <cstdlib>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <ILMR/config_ini.hh>
#include <ILMR/joystick.hh>
#include <ILMR/keyboard.hh>
#include <ILMR/logger.hh>
#include <ILMR/px4_device.hh>

#include <CLI/CLI.hpp>

/*
 * Wait for Joystick input, non-blocking implementation using STL
 */
JoystickEvent
joystick_event_handler(Joystick& joystick,
                       std::vector<std::shared_ptr<Px4Device>> iris_x,
                       float speed,
                       std::shared_ptr<spdlog::logger> logger,
                       int id)
{
  while (true) {
    // Attempt to read an event from the joystick
    JoystickEvent event;
    if (joystick.read_event(&event)) {

      if (joystick.ButtonAChanged(event)) {
        iris_x.at(id)->arm();
        logger->info("arming...\n");

        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else if (joystick.ButtonBChanged(event)) {
        iris_x.at(id)->land();
        logger->info("landing...\n");

      } else if (joystick.ButtonXChanged(event)) {
        iris_x.at(id)->takeoff(1);
        logger->info("taking off...\n");
        std::this_thread::sleep_for(std::chrono::seconds(5));

      } else if (joystick.ButtonYChanged(event)) {
        iris_x.at(id)->init_speed();
        iris_x.at(id)->start_offboard_mode();
        logger->info("Start offoard mode...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));

      } else if (joystick.ButtonL1Changed(event)) {
        logger->info("L1");

      } else if (joystick.ButtonR1Changed(event)) {
        logger->info("R1");

      } else if (joystick.ButtonSelectChanged(event)) {
        logger->info("Select");

      } else if (joystick.ButtonStartChanged(event)) {
        logger->info("Start");

      } else if (joystick.ButtonGuideChanged(event)) {
        logger->info("Guide");

      } else if (joystick.RightAxisXChanged(event)) {

        if (joystick.RightAxisXChanged(event) > 0) {
          /* Speed should be function of the value of joystick  */
          iris_x.at(id)->right(speed);
          logger->info("Moving right...\n");

        } else {
          /* Speed should be function of the value of joystick  */
          iris_x.at(id)->left(speed);
          logger->info("Moving left...\n");
        }
      } else if (joystick.RightAxisYChanged(event)) {

        if (joystick.RightAxisYChanged(event) > 0) {
          /* Speed should be function of the value of joystick  */
          iris_x.at(id)->backward(speed);
          logger->info("Moving backward...\n");

        } else {
          /* Speed should be function of the value of joystick  */
          /*  Speed should be fixed as the joystick does not move */
          iris_x.at(id)->forward(speed);
          logger->info("Moving forward...\n");
        }

      } else if (joystick.LeftAxisXChanged(event)) {

        if (joystick.LeftAxisXChanged(event) > 0) {

          iris_x.at(id)->turnToLeft();
          logger->info("Turn to left...\n");

        } else {
          iris_x.at(id)->turnToRight();
          logger->info("Turn to right...\n");
        }

      } else if (joystick.LeftAxisYChanged(event)) {

        if (joystick.LeftAxisYChanged(event) > 0) {
          /* Speed should be function of the value of joystick  */
          iris_x.at(id)->down(speed);
          logger->info("Moving down...\n");

        } else {
          /* Speed should be function of the value of joystick  */
          iris_x.at(id)->up(speed);
          logger->info("Moving up...\n");
        }
      } else if (joystick.AxisL2Changed(event)) {
        logger->info("L2: {}", joystick.AxisL2Changed(event));

      } else if (joystick.AxisR2Changed(event)) {
        logger->info("R2: {}", joystick.AxisR2Changed(event));

      } else if (joystick.DpadXChanged(event)) {
        logger->info("Dx: {}", joystick.DpadXChanged(event));

      } else if (joystick.DpadYChanged(event)) {
        logger->info("Dy: {}", joystick.DpadYChanged(event));
      }
    }
  }
}

void
keyboard_event_handler(std::vector<std::shared_ptr<Px4Device>> iris_x,
                       float speed,
                       std::shared_ptr<spdlog::logger> logger,
                       int id)
{
  Keyboard keyboard(STDIN_FILENO);
  while (true) {
    // Attempt to read an event from the joystick
    int ch = keyboard.poll_event(STDIN_FILENO);

    switch (ch) {
      case 'm':
        iris_x.at(id)->arm();
        logger->info("Arming...\n");
        break;
      case 'l':
        iris_x.at(id)->land();
        logger->info("Landing...\n");
        break;
      case 't':
        iris_x.at(id)->takeoff(1);
        logger->info("Taking off...\n");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        break;
      case 'o':
        iris_x.at(id)->init_speed();
        iris_x.at(id)->start_offboard_mode();
        logger->info("Start offoard mode...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        break;
      case 'd':
        iris_x.at(id)->right(speed);
        logger->info("Moving right...\n");
        break;
      case 'a':
        iris_x.at(id)->left(speed);
        logger->info("Moving left...\n");
        break;
      case 's':
        iris_x.at(id)->backward(speed);
        logger->info("Moving backward...\n");
        break;
      case 'w':
        iris_x.at(id)->forward(speed);
        logger->info("Moving forward...\n");
        break;
      case 'z':
        iris_x.at(id)->down(speed);
        logger->info("Moving down...\n");
        break;
      case 'q':
        iris_x.at(id)->up(speed);
        logger->info("Moving up...\n");
        break;
      case static_cast<int>(Keyboard::Special_keys::CTRL_C):
        keyboard.disable_raw_mode(STDIN_FILENO);
        exit(0);
      default:
        logger->info("NON assigned key: {}", ch);
        break;
    }
  }
}

/*  Print the possible keyboard or joystick input */
void
usage(std::ostream& out)
{
  out << "Usage : " << std ::endl
      << "To fly a quadrotor manually you need to start the software with "
         "flying option"
      << std::endl
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
      << " X : takeoff " << std::endl
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
int
main(int argc, char* argv[])
{
  /*  Init configs */
  Configs configs("/meta/lemon/quad.ini");

  CLI::App app{
    "This example allow to control a quadrotor using keyboard or joystick.\n"
    "The example works on real quadrotors and in simlation.\n"
    "Users can type ./manual_control -h to learn keyboard layout.\n"
    "Enter i = 0 to control only one real quadrotor using keyboard or "
    "joystick.\n"
  };
  int id = 0;
  app.add_option("-i", id, "Enter the id of the quadrotor to control");
  CLI11_PARSE(app, argc, argv);

  auto logger = ILMR::logger::init();
  ILMR::logger::create_library_logger(logger);

  bool joystick_mode = true;
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound()) {
    logger->error("No device found, please connect a joystick. \n");
    logger->error(
      "Joystick mode disabled. Control only possible using a keyboard.\n");
    joystick_mode = false;
  }

  std::vector<lt::port_type> ports = configs.quads_ports();

  /* Create a vector of controllers. Each controller connect to one
   * quadcopters at a time
   */
  std::vector<std::shared_ptr<Px4Device>> iris_x;
  for (auto& it : ports) {
    iris_x.push_back(std::make_shared<Px4Device>("udp", it));
    logger->info("Add an iris quadrotor!\n");
  }

  logger->info("Ports number: {}", ports);

  auto joystick_handler = [&]() {
    if (joystick_mode) {
      joystick_event_handler(
        joystick, iris_x, configs.speed(), logger, id);
    }
  };

  auto keyboard_handler = [&]() {
    keyboard_event_handler(
      iris_x, configs.speed(), logger, id);
  };

  auto joystick_events = std::async(std::launch::async, joystick_handler);
  auto keyboard_events = std::async(std::launch::async, keyboard_handler);

  joystick_events.get();
  keyboard_events.get();
}
