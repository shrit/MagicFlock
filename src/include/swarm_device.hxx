#pragma once

#include "swarm_device.hh"

template<class flight_controller_t>
SwarmDevice<flight_controller_t>::SwarmDevice(
  std::vector<std::shared_ptr<flight_controller_t>> quads)
  : iris_x_(std::move(quads))
{}

template<class flight_controller_t>
void
SwarmDevice<flight_controller_t>::one_quad_execute_trajectory(
  unsigned int id,
  Actions::Action action,
  int speed,
  unsigned int milliseconds)
{
  if (action == Actions::Action::left) {
    iris_x_.at(id)->left(speed, milliseconds);

  } else if (action == Actions::Action::right) {
    iris_x_.at(id)->right(speed, milliseconds);

  } else if (action == Actions::Action::forward) {
    iris_x_.at(id)->forward(speed, milliseconds);

  } else if (action == Actions::Action::backward) {
    iris_x_.at(id)->backward(speed, milliseconds);

  } else if (action == Actions::Action::up) {
    iris_x_.at(id)->up(speed, milliseconds);

  } else if (action == Actions::Action::down) {
    iris_x_.at(id)->down(speed, milliseconds);
  }
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::arm()
{
  bool arm = true;
  std::vector<std::thread> threads;

  threads.push_back(std::thread([&]() { arm = iris_x_.at(0)->arm(); }));

  threads.push_back(std::thread([&]() { arm = iris_x_.at(1)->arm(); }));

  threads.push_back(std::thread([&]() { arm = iris_x_.at(2)->arm(); }));

  threads.push_back(std::thread([&]() { arm = iris_x_.at(3)->arm(); }));  

  for (auto& thread : threads) {
    thread.join();
  }

  if (!arm)
    return false;

  return true;
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::arm_specific_quadrotor(unsigned int id)
{
  bool arm = iris_x_.at(id)->arm();
  if (!arm)
    return false;

  return true;
}

template<class flight_controller_t>
void
SwarmDevice<flight_controller_t>::init_speed()
{
  for (auto it : iris_x_) {
    it->init_speed();
  }
}

template<class flight_controller_t>
void
SwarmDevice<flight_controller_t>::init_speed_specific_quadrotor(unsigned int id)
{
  iris_x_.at(id)->init_speed();
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::start_offboard_mode()
{
  bool offboard_mode;
  for (auto it : iris_x_) {
    offboard_mode = it->start_offboard_mode();
    if (!offboard_mode)
      return false;
  }
  return true;
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::start_offboard_mode_specific_quadrotor(
  unsigned int id)
{
  bool offboard_mode = iris_x_.at(id)->start_offboard_mode();
  if (!offboard_mode)
    return false;

  return true;
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::takeoff(float meters)
{
  bool takeoff = true;
  std::vector<std::thread> threads;

  threads.push_back(
    std::thread([&]() { takeoff = iris_x_.at(0)->takeoff(meters); }));

  threads.push_back(
    std::thread([&]() { takeoff = iris_x_.at(1)->takeoff(meters); }));

  threads.push_back(
    std::thread([&]() { takeoff = iris_x_.at(2)->takeoff(meters); }));

  threads.push_back(
    std::thread([&]() { takeoff = iris_x_.at(3)->takeoff(meters); }));  

  for (auto& thread : threads) {
    thread.join();
  }
  if (!takeoff)
    return false;

  return true;
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::takeoff_specific_quadrotor(float meters,
                                                             unsigned int id)
{
  bool takeoff = iris_x_.at(id)->takeoff(meters);
  if (!takeoff)
    return false;

  return true;
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::land()
{
  bool land = true;
  std::vector<std::thread> threads;

  threads.push_back(std::thread([&]() { land = iris_x_.at(0)->land(); }));

  threads.push_back(std::thread([&]() { land = iris_x_.at(1)->land(); }));

  threads.push_back(std::thread([&]() { land = iris_x_.at(2)->land(); }));

  threads.push_back(std::thread([&]() { land = iris_x_.at(3)->land(); }));  

 for (auto& thread : threads) {
    thread.join();
  }
  if (!land)
    return false;

  return true;
}

template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::land_specific_quadrotor(unsigned int id)
{
  bool land = iris_x_.at(id)->land();
  if (!land)
    return false;

  return true;
}

template<class flight_controller_t>
std::vector<lt::position_GPS<double>>
SwarmDevice<flight_controller_t>::positions_GPS()
{
  std::vector<lt::position_GPS<double>> positions;
  for (auto i : iris_x_) {
    positions.push_back(i->get_position_GPS());    
  }
  return positions;
}

/* This function do arming, takoff, and offboard mode for a swarm*/
template<class flight_controller_t>
bool
SwarmDevice<flight_controller_t>::in_air(float meters)
{
  bool start_episode = true;
  bool arm = this->arm();
  if (!arm) {
    start_episode = false;
  } else if (arm) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = this->takeoff(meters);
    if (!takeoff) {
      start_episode = false;
    } else if (takeoff) {
      /*  Setting up speed in order to switch the mode */
      this->init_speed();
      /*  Switch to offboard mode, Allow the control */
      /* Stop the episode is the one quadcopter have fallen to set
         offbaord mode */
      bool offboard_mode = this->start_offboard_mode();
      if (!offboard_mode)
        start_episode = false;

      /*  Wait to complete the take off process */
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  return start_episode;
}
