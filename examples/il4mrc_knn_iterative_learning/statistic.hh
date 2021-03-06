#pragma once

/*  Standard C++ includes  */
# include <chrono>
# include <thread>
# include <vector>

/* local includes  */
# include <ILMR/knn_predictor.hh>
# include <ILMR/quadrotor.hh>
# include <ILMR/swarm_device.hh>
# include <ILMR/action.hh>
# include <ILMR/log.hh>
# include <ILMR/logger.hh>
# include <ILMR/time_steps.hh>

using namespace ILMR;

template<class flight_controller_t,
	 class simulator_t>
class Statistic
{

public:
  Statistic(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
            const std::vector<Quadrotor<simulator_t>>& quadrotors,
            std::shared_ptr<simulator_t> gzs,
            std::shared_ptr<spdlog::logger> logger);

  void generate_trajectory_using_model(bool random_leader_action,
                                       bool stop_going_down);

  void run();

  Statistic(Statistic const&) = delete;
  Statistic(Statistic &&) = default;

private:

  TimeSteps time_steps_;
  int episode_;
  int max_episode_ ;
  std::shared_ptr<simulator_t> sim_interface_;
  bool start_episode_;
  SwarmDevice<flight_controller_t> swarm_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;
  std::shared_ptr<spdlog::logger> logger_;

  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_2_;
};

# include "statistic.hxx"
