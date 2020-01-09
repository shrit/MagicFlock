#pragma once

/*  Standard C++ includes  */
#include <vector>

/* Local includes  */
#include <ILMR/action.hh>
#include <ILMR/data_set.hh>
#include <ILMR/global.hh>
#include <ILMR/logger.hh>
#include <ILMR/quadrotor.hh>
#include <ILMR/swarm_device.hh>

namespace lt = local_types;

template<class flight_controller_t, class simulator_t>
class TrajectoryNoise
{

public:
  TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
                  const std::vector<Quadrotor<simulator_t>>& quadrotors,
                  std::shared_ptr<simulator_t> sim_interface,
                  std::shared_ptr<spdlog::logger> logger);
  void run();

  void test_trajectory(bool stop_down_action);

  TrajectoryNoise(TrajectoryNoise const&) = delete;
  TrajectoryNoise(TrajectoryNoise&&) = default;

private:
  int count_;
  DataSet<simulator_t> data_set_;
  int episode_;
  int max_episode_;
  Math_tools mtools_;
  Actions action_;
  std::shared_ptr<simulator_t> sim_interface_;
  bool start_episode_;
  SwarmDevice<flight_controller_t> swarm_;
  std::vector<Quadrotor<simulator_t>> quadrotors_;
  std::shared_ptr<spdlog::logger> logger_;  
  typename std::vector<Quadrotor<simulator_t>>::iterator leader_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_1_;
  typename std::vector<Quadrotor<simulator_t>>::iterator follower_2_;

  /*  This is a monster ! */
  std::vector<double> forward_action_vec_;
  std::vector<double> backward_action_vec_;
  std::vector<double> left_action_vec_;
  std::vector<double> right_action_vec_;
  std::vector<double> up_action_vec_;
  std::vector<double> down_action_vec_;

  std::vector<double> f_k_l_action_vec_; /*  Forward action known that left
                                            action has been executed */
  std::vector<double> f_k_r_action_vec_; /*  Forward action known that right
                                            action has been executed */
  std::vector<double> f_k_u_action_vec_; /*  Forward action known that up action
                                            has been executed */
  std::vector<double> f_k_d_action_vec_; /*  Forward action known that down
                                            action has been executed */

  std::vector<double> b_k_l_action_vec_; /*  Backward action known that left
                                            action has been executed */
  std::vector<double> b_k_r_action_vec_; /*  Backward action known that right
                                            action has been executed */
  std::vector<double> b_k_u_action_vec_; /*  Backward action known that up
                                            action has been executed */
  std::vector<double> b_k_d_action_vec_; /*  Backward action known that down
                                            action has been executed */

  std::vector<double> l_k_f_action_vec_; /*  Left action known that forward
                                            action has been executed */
  std::vector<double> l_k_b_action_vec_; /*  Left action known that backward
                                            action has been executed */
  std::vector<double>
    l_k_u_action_vec_; /*  Left action known that up action has been executed */
  std::vector<double> l_k_d_action_vec_; /*  Left action known that down action
                                            has been executed */

  std::vector<double> r_k_f_action_vec_; /*  Right action known that forward
                                            action has been executed */
  std::vector<double> r_k_b_action_vec_; /*  Right action known that backward
                                            action has been executed */
  std::vector<double> r_k_u_action_vec_; /*  Right action known that up action
                                            has been executed */
  std::vector<double> r_k_d_action_vec_; /*  Right action known that down action
                                            has been executed */

  std::vector<double> u_k_f_action_vec_; /*  up action known that forward action
                                            has been executed */
  std::vector<double> u_k_b_action_vec_; /*  up action known that backward
                                            action has been executed */
  std::vector<double>
    u_k_l_action_vec_; /*  up action known that left action has been executed */
  std::vector<double> u_k_r_action_vec_; /*  up action known that right action
                                            has been executed */

  std::vector<double> d_k_f_action_vec_; /*  down action known that forward
                                            action has been executed */
  std::vector<double> d_k_b_action_vec_; /*  down action known that backward
                                            action has been executed */
  std::vector<double> d_k_l_action_vec_; /*  down action known that left action
                                            has been executed */
  std::vector<double> d_k_r_action_vec_; /*  down action known that right action
                                            has been executed */
};

#include "trajectory_noise.hxx"
