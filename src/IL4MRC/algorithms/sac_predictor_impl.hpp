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
             ReplayType>::SACPredictor(QuadrotorType& quad)
  : replayMethod_(32, 10000, 1, 12)
  , quadrotor_(quad)
  , policyNetwork_(mlpack::ann::EmptyLoss<>(),
                   mlpack::ann::GaussianInitialization(0, 0.1))
  , qNetwork_(mlpack::ann::EmptyLoss<>(),
              mlpack::ann::GaussianInitialization(0, 0.1))
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
  mlpack::rl::ContinuousActionEnv::State::dimension = 12;
  mlpack::rl::ContinuousActionEnv::Action::size = 3;

  policyNetwork_.Add<mlpack::ann::Linear<>>(12, 200);
  policyNetwork_.Add<mlpack::ann::ReLULayer<>>();
  policyNetwork_.Add<mlpack::ann::Linear<>>(200, 200);
  policyNetwork_.Add<mlpack::ann::ReLULayer<>>();
  policyNetwork_.Add<mlpack::ann::Dropout<>>(0.5);
  policyNetwork_.Add<mlpack::ann::Linear<>>(200, 200);
  policyNetwork_.Add<mlpack::ann::ReLULayer<>>();
  policyNetwork_.Add<mlpack::ann::Dropout<>>(0.5);
  policyNetwork_.Add<mlpack::ann::Linear<>>(200, 3);
  policyNetwork_.Add<mlpack::ann::TanHLayer<>>();
  policyNetwork_.ResetParameters();

  qNetwork_.Add<mlpack::ann::Linear<>>(12 + 3, 200);
  qNetwork_.Add<mlpack::ann::ReLULayer<>>();
  qNetwork_.Add<mlpack::ann::Linear<>>(200, 200);
  qNetwork_.Add<mlpack::ann::ReLULayer<>>();
  qNetwork_.Add<mlpack::ann::Dropout<>>(0.5);
  qNetwork_.Add<mlpack::ann::Linear<>>(200, 200);
  qNetwork_.Add<mlpack::ann::ReLULayer<>>();
  qNetwork_.Add<mlpack::ann::Dropout<>>(0.5);
  qNetwork_.Add<mlpack::ann::Linear<>>(200, 1);
  qNetwork_.ResetParameters();

  config_.TargetNetworkSyncInterval() = 1;
  config_.UpdateInterval() = 1;

  agent_ = std::make_unique<
  mlpack::rl::SAC<EnvironmentType, NetworkType, UpdaterType, PolicyType>>(
    config_, qNetwork_, policyNetwork_, replayMethod_);

  agent_->Deterministic() = false;
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
             ReplayType>::
  train(size_t& consecutiveEpisodes,
        const size_t numSteps,
        std::function<void()> execute_action,
        std::function<double()> evaluate_reward,
        std::function<bool()> examine_environment)
{
  // Set up the state and action space.
  mlpack::rl::ContinuousActionEnv::State::dimension = 12;
  mlpack::rl::ContinuousActionEnv::Action::size = 3;
  logger::logger_->info("Training for: {}", numSteps, " steps.");
  while (agent_->TotalSteps() < numSteps) {
    episodeReturn_ = 0;
    bool isTerminal = false;
    do {
      agent_->State().Data() = quadrotor_.current_state().Data();
      logger::logger_->info("Quadrotor State {}",
                            quadrotor_.current_state().Data());
      logger::logger_->info("Agent State {}", agent_->State().Data());

      // Here we need to execute action, swarm_.one..etc
      agent_->SelectAction();
      arma::colvec action = arma::conv_to<arma::colvec>::from(agent_->Action().action);
      logger::logger_->info("The created action {}", action);
      quadrotor_.current_action().set_action(action);

      execute_action();
      mlpack::rl::ContinuousActionEnv::State
        nextState; // replace it with an arma matrix
      nextState.Data() = quadrotor_.current_state().Data();
      logger::logger_->info("Agent NextState {}", nextState.Data());
      double reward = evaluate_reward();
      logger::logger_->info("Print reward value", reward);
      isTerminal = examine_environment();

      replayMethod_.Store(
        agent_->State(), agent_->Action(), reward, nextState, isTerminal, 0.99);
      episodeReturn_ += reward;
      agent_->TotalSteps()++;

      if (agent_->Deterministic() ||
          agent_->TotalSteps() < config_.ExplorationSteps())
        continue;

      for (size_t i = 0; i < config_.UpdateInterval(); i++)
        agent_->Update();
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
    //             << "\\t Total steps: " << agent_->TotalSteps() << std::endl;
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
