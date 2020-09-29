#pragma once

template<class QuadrotorType>
SwarmDevice<QuadrotorType>::SwarmDevice(std::vector<QuadrotorType>& quads)
  : quads_(quads)
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
    quads_.at(id).controller_->left(speed, milliseconds);

  } else if (action == DiscretActions::Action::right) {
    quads_.at(id).controller_->right(speed, milliseconds);

  } else if (action == DiscretActions::Action::forward) {
    quads_.at(id).controller_->forward(speed, milliseconds);

  } else if (action == DiscretActions::Action::backward) {
    quads_.at(id).controller_->backward(speed, milliseconds);

  } else if (action == DiscretActions::Action::up) {
    quads_.at(id).controller_->up(speed, milliseconds);

  } else if (action == DiscretActions::Action::down) {
    quads_.at(id).controller_->down(speed, milliseconds);
  }
}
template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::one_quad_execute_trajectory(
  unsigned int id,
  ContinuousActions action)
{
  ignition::math::Vector3d vec = action.action();
  logger::logger_->info("Executed Vec: {}", vec);
  quads_.at(id).controller_->set_velocity_vector(vec);
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::arm()
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto it : quads_) {
    threads.emplace_back(
      std::thread([&]() { results.emplace_back(it.controller_->arm()); }));
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
void
SwarmDevice<QuadrotorType>::arm_async()
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread([&]() {
      it.controller_->arm_async();
      // arming_results_.emplace_back(it.controller_->arm_result());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::arm_specific_quadrotor(unsigned int id)
{
  bool arm = quads_.at(id).controller_->arm();
  if (!arm)
    return false;

  return true;
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::init_speed()
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread([&]() { it.controller_->init_speed(); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::init_speed_specific_quadrotor(unsigned int id)
{
  quads_.at(id).controller_->init_speed();
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::start_offboard_mode()
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread(
      [&]() { results.emplace_back(it.controller_->start_offboard_mode()); }));
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
void
SwarmDevice<QuadrotorType>::start_offboard_mode_async()
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread([&]() {
      it.controller_->start_offboard_mode_async();
      // offboard_mode_results_.emplace_back(it.controller_->start_offboard_result());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::stop_offboard_mode_async()
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(
      std::thread([&]() { it.controller_->stop_offboard_mode_async(); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::start_offboard_mode_specific_quadrotor(
  unsigned int id)
{
  bool offboard_mode = quads_.at(id).controller_->start_offboard_mode();
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
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread(
      [&]() { results.emplace_back(it.controller_->takeoff(meters)); }));
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
void
SwarmDevice<QuadrotorType>::takeoff_async(float meters)
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread([&]() {
      it.controller_->takeoff_async(meters);
      // takeoff_results_.emplace_back(it.controller_->takeoff_result());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::takeoff_specific_quadrotor(float meters,
                                                       unsigned int id)
{
  bool takeoff = quads_.at(id).controller_->takeoff(meters);
  if (!takeoff)
    return false;

  return true;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::land()
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto&& it : quads_) {
    threads.emplace_back(
      std::thread([&]() { results.emplace_back(it.controller_->land()); }));
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
void
SwarmDevice<QuadrotorType>::land_async()
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread([&]() {
      it.controller_->land_async();
      // landed_results_.emplace_back(it.controller_->land_result());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::flight_mode_async()
{
  std::vector<std::thread> threads;
  std::vector<bool> results;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread(
      [&]() { results.emplace_back(it.controller_->flight_mode()); }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::landed_state_async()
{
  std::vector<std::thread> threads;
  for (auto&& it : quads_) {
    threads.emplace_back(std::thread([&]() {
      landed_state_results_.emplace_back(it.controller_->landed_state());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
template<class status>
bool
SwarmDevice<QuadrotorType>::checking_status(std::vector<status> results)
{
  bool recheck = false;
  if (std::any_of(results.begin(), results.end(), [](status value) {
        if (value != status::Success) {
          logger::logger_->error("STATUS", value);
          return value;
        }
      })) {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    recheck = true;
  }
  return recheck;
}

template<class QuadrotorType>
template<class status>
bool
SwarmDevice<QuadrotorType>::check_landed_state(std::vector<status> results)
{
  bool recheck = false;
  if (std::any_of(results.begin(), results.end(), [](status value) {
        if (value != status::TakingOff) {
          logger::logger_->error("STATUS", value);
          return value;
        }
      })) {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    recheck = true;
  }
  return recheck;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::land_specific_quadrotor(unsigned int id)
{
  bool land = quads_.at(id).controller_->land();
  if (!land)
    return false;

  return true;
}

template<class QuadrotorType>
std::vector<position_GPS<double>>
SwarmDevice<QuadrotorType>::positions_GPS()
{
  std::vector<position_GPS<double>> positions;
  for (auto&& i : quads_) {
    positions.push_back(i.get_position_GPS());
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

/* This function do arming, takoff, and offboard mode for a swarm*/
template<class QuadrotorType>
void
SwarmDevice<QuadrotorType>::in_air_async(float meters)
{
  /* Need to check the landed state after each one, and
   * wait until it is in air or every one.
   * We should not wait infinite time, only some amount
   * of time, otherwise land everyone and start from new
   * */
  this->arm_async();
  /* Check Arming for all quadrotors */
  // if (!checking_status(arming_results_)) {
  //   // Recheck after 10 seconds if things were not well;
  //   checking_status(arming_results_);
  // }

  /* Stop the episode if one of the quad has fallen to takoff */
  this->takeoff_async(meters);
  /* Check the takeoff for all the quadrotors*/
  // if (!checking_status(takeoff_results_)) {
  //   // Recheck after 10 seconds if things were not well;
  //   checking_status(takeoff_results_);
  // }

  std::this_thread::sleep_for(std::chrono::seconds(10));
  // Check if quadrotors have finished taking off
  // if (!check_landed_state(landed_state_results_)) {
  //   // Recheck after at least 10 seconds
  //   check_landed_state(landed_state_results_);
  // }
  /*  Setting up speed in order to switch the mode */
  this->init_speed();
  /* Check the init speed status*/

  /* Switching mode is required in order to activate*/
  this->start_offboard_mode_async();
  // if (!checking_status(offboard_mode_results_)) {
  //   // Recheck after 10 seconds if things were not well;
  //   checking_status(offboard_mode_results_);
  // } /*  Wait to complete the take off process */
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::examin_swarm_shape()
{
  bool shape = shape_.is_good_shape(quads_);
  return shape;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::examin_swarm_shape(double lower_threshold,
                                               double upper_threshold)
{
  shape_.lower_threshold() = lower_threshold;
  shape_.upper_threshold() = upper_threshold;
  bool shape = shape_.is_good_shape(quads_);
  return shape;
}

template<class QuadrotorType>
bool
SwarmDevice<QuadrotorType>::examin_destination(
  const ignition::math::Vector3d& destination)
{
  dest_.destination() = destination;
  bool dest = dest_.has_arrived(quads_);
  return dest;
}
