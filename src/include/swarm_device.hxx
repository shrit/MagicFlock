#pragma once

template<class QuadrotorType>
SwarmDevice<QuadrotorType>::SwarmDevice(
  std::vector<std::shared_ptr<QuadrotorType>> quads)
  : quads_(std::move(quads))
{
  // Nothing to do here
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::one_quad_execute_trajectory(
  unsigned int id,
  DiscretActions::Action action,
  int speed,
  unsigned int milliseconds)
{
  if (action == DiscretActions::Action::left) {
    quads_.at(id)->controller()->left(speed, milliseconds);

  } else if (action == DiscretActions::Action::right) {
    quads_.at(id)->controller()->right(speed, milliseconds);

  } else if (action == DiscretActions::Action::forward) {
    quads_.at(id)->controller()->forward(speed, milliseconds);

  } else if (action == DiscretActions::Action::backward) {
    quads_.at(id)->controller()->backward(speed, milliseconds);

  } else if (action == DiscretActions::Action::up) {
    quads_.at(id)->controller()->up(speed, milliseconds);

  } else if (action == DiscretActions::Action::down) {
    quads_.at(id)->controller()->down(speed, milliseconds);
  }
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::one_quad_execute_trajectory(
  unsigned int id,
  ContinuousActions action)
{
  ignition::math::Vector3d vec = action.velocity_vector();
  quads_.at(id)->controller()->set_velocity_vector(vec);
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::arm()
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto it : quads_) {
    threads.emplace_back(
      std::thread([&]() { results.emplace_back(it->controller()->arm()); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }

  if (std::any_of(results.begin(), results.end(), [](bool value) {
        return value == false;
      }))
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::arm_specific_quadrotor(unsigned int id)
{
  bool arm = quads_.at(id)->controller()->arm();
  if (!arm)
    return false;

  return true;
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::init_speed()
{
  std::vector<std::thread> threads;
  for (auto it : quads_) {
    threads.emplace_back(
      std::thread([&]() { it->controller()->init_speed(); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::init_speed_specific_quadrotor(unsigned int id)
{
  quads_.at(id)->controller()->init_speed();
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::start_offboard_mode()
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto it : quads_) {
    threads.emplace_back(std::thread([&]() {
      results.emplace_back(it->controller()->start_offboard_mode());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }

  if (std::any_of(results.begin(), results.end(), [](bool value) {
        return value == false;
      }))
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::start_offboard_mode_specific_quadrotor(
  unsigned int id)
{
  bool offboard_mode = quads_.at(id)->controller()->start_offboard_mode();
  if (!offboard_mode)
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::takeoff(float meters)
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto it : quads_) {
    threads.emplace_back(std::thread(
      [&]() { results.emplace_back(it->controller()->takeoff(meters)); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
  if (std::any_of(results.begin(), results.end(), [](bool value) {
        return value == false;
      }))
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::takeoff_specific_quadrotor(float meters,
                                                       unsigned int id)
{
  bool takeoff = quads_.at(id)->controller()->takeoff(meters);
  if (!takeoff)
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::land()
{
  bool land = true;
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto it : quads_) {
    threads.emplace_back(
      std::thread([&]() { results.emplace_back(it->controller()->land()); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
  if (std::any_of(results.begin(), results.end(), [](bool value) {
        return value == false;
      }))
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::land_specific_quadrotor(unsigned int id)
{
  bool land = quads_.at(id)->controller()->land();
  if (!land)
    return false;

  return true;
}

template<class QuadrotorType>
std::vector<position_GPS<double>>
SwarmDevice<QuadrotorType>::positions_GPS()
{
  std::vector<position_GPS<double>> positions;
  for (auto i : quads_) {
    positions.push_back(i->get_position_GPS());
  }
  return positions;
}

/* This function do arming, takoff, and offboard mode for a swarm*/
template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::in_air(float meters)
{
  bool start_episode = true;
  bool arm = this->arm();
  if (!arm) {
    start_episode = false;
  } else {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = this->takeoff(meters);
    if (!takeoff) {
      start_episode = false;
    } else {
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

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::examin_swarm_shape()
{
  bool shape = shape_.is_good_shape(quads_);
  return shape;
}
