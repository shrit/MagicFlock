#pragma once

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename QuadrotorType,
         typename ReplayType>
SACPredictor<EnvironmentType,
             NetworkType,
             UpdaterType,
             PolicyType,
             QuadrotorType,
             ReplayType>::SACPredictor(const QuadrotorType& quad)
  : replayMethod_(32, 10000)
  , quadrotor_(quad)
{
  // Nothing to do here.
}

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename QuadrotorType,
         typename ReplayType>
void
SACPredictor<EnvironmentType,
             NetworkType,
             UpdaterType,
             PolicyType,
             QuadrotorType,
             ReplayType>::SacNetwork()
{
  // Set up the actor and critic networks.
  mlpack::ann::FFN<mlpack::ann::EmptyLoss<>,
                   mlpack::ann::GaussianInitialization>
    policyNetwork(mlpack::ann::EmptyLoss<>(),
                  mlpack::ann::GaussianInitialization(0, 0.1));
  policyNetwork.Add(new mlpack::ann::Linear<>(
    mlpack::rl::ContinuousActionEnv::State::dimension, 32));
  policyNetwork.Add(new mlpack::ann::ReLULayer<>());
  policyNetwork.Add(new mlpack::ann::Linear<>(
    32, mlpack::rl::ContinuousActionEnv::Action::size));
  policyNetwork.Add(new mlpack::ann::TanHLayer<>());

  mlpack::ann::FFN<mlpack::ann::EmptyLoss<>,
                   mlpack::ann::GaussianInitialization>
    qNetwork(mlpack::ann::EmptyLoss<>(),
             mlpack::ann::GaussianInitialization(0, 0.1));
  qNetwork.Add(new mlpack::ann::Linear<>(
    mlpack::rl::ContinuousActionEnv::State::dimension +
      mlpack::rl::ContinuousActionEnv::Action::size,
    32));
  qNetwork.Add(new mlpack::ann::ReLULayer<>());
  qNetwork.Add(new mlpack::ann::Linear<>(32, 1));

  // Set up training configurations.
  config_.TargetNetworkSyncInterval() = 1;
  config_.UpdateInterval() = 1;

  //agent_(config_, qNetwork, policyNetwork, replayMethod_);
  agent_.Deterministic() = false;
}

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename QuadrotorType,
         typename ReplayType>
void
SACPredictor<EnvironmentType,
             NetworkType,
             UpdaterType,
             PolicyType,
             QuadrotorType,
             ReplayType>::train(size_t& consecutiveEpisodes,
                                const size_t numSteps,
                                std::function<void(void)> execute_action,
                                std::function<double(void)> evaluate_reward,
                                std::function<double(void)> examine_environment)
{
  logger::logger_->info("Training for: {}", numSteps, " steps.");
  while (agent_.TotalSteps() < numSteps) {
    episodeReturn_ = 0;
    bool isTerminal = false;
    do {
      agent_.State().Data() = quadrotor_.current_state().Data();

      // Here we need to execute action, swarm_.one..etc
      agent_.SelectAction();
      arma::mat action = { double(agent_.Action().action[0] * 2) };
      quadrotor_.current_action() = action;

      execute_action();
      mlpack::rl::ContinuousActionEnv::State
        nextState; // replace it with an arma matrix
      nextState.Data() = quadrotor_.current_state().Data();
      // 1) we need to store the executed action in agent_.Action()
      // Thus we need to execute a lamda function here in order to
      // recover the values of the function
      // 2) We need to create a reward function to store it
      // 3) We need to uderstand env.done in order to replicate it
      // 4)
      double reward = evaluate_reward();
      isTerminal = examine_environment();
      replayMethod_.Store(
        agent_.State(), agent_.Action(), reward, nextState, isTerminal, 0.99);
      episodeReturn_ += reward;
      agent_.TotalSteps()++;

      if (agent_.Deterministic() ||
          agent_.TotalSteps() < config_.ExplorationSteps())
        continue;

      for (size_t i = 0; i < config_.UpdateInterval(); i++)
        agent_.Update();
    }

    while (!isTerminal);
    returnList_.push_back(episodeReturn_);

    if (returnList_.size() > consecutiveEpisodes)
      returnList_.erase(returnList_.begin());

    double averageReturn =
      std::accumulate(returnList_.begin(), returnList_.end(), 0.0) /
      returnList_.size();
    // if (episodes % 4 == 0) {
    //   std::cout << "Avg return in last " << returnList_.size()
    //             << " episodes: " << averageReturn
    //             << "\\t Episode return: " << episodeReturn_
    //             << "\\t Total steps: " << agent_.TotalSteps() << std::endl;
    // }
    if (isTerminal)
      break;
  }
}

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename QuadrotorType,
         typename ReplayType>
void
SACPredictor<EnvironmentType,
             NetworkType,
             UpdaterType,
             PolicyType,
             QuadrotorType,
             ReplayType>::test()
{}
