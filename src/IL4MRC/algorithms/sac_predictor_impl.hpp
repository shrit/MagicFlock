#pragma once

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename ReplayType = RandomReplay<EnvironmentType>,
         typename QuadrotorType>
SACPredictor<QuadrotorType>::SACPredictor(const QuadrotorType& quad)
{
  // Nothing to do here.
}

template<class QuadrotorType>
void
SACPredictor<QuadrotorType>::SacNetwork()
{
  // Set up the actor and critic networks.
  mlpack::ann::FFN<EmptyLoss<>, mlpack::ann::GaussianInitialization>
    policyNetwork(EmptyLoss<>(), GaussianInitialization(0, 0.1));
  policyNetwork.Add(new Linear<>(State::dimension, 32));
  policyNetwork.Add(new ReLULayer<>());
  policyNetwork.Add(new Linear<>(32, Action::size));
  policyNetwork.Add(new TanHLayer<>());

  mlpack::ann::FFN<EmptyLoss<>, mlpack::ann::GaussianInitialization> qNetwork(
    EmptyLoss<>(), GaussianInitialization(0, 0.1));
  qNetwork.Add(new Linear<>(State::dimension + Action::size, 32));
  qNetwork.Add(new ReLULayer<>());
  qNetwork.Add(new Linear<>(32, 1));

  // Set up the policy method.
  RandomReplay<ContinuousActionEnv> replayMethod(32, 10000);

  // Set up training configurations.
  config_.TargetNetworkSyncInterval() = 1;
  config_.UpdateInterval() = 1;

  // Set up Soft actor-critic agent.
  SAC<ContinuousActionEnv,
      decltype(qNetwork),
      decltype(policyNetwork),
      AdamUpdate>
    agent(config_, qNetwork, policyNetwork, replayMethod);
}

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename ReplayType = RandomReplay<EnvironmentType>>
template<class QuadrotorType>
void
SACPredictor<QuadrotorType>::train(
  SAC<EnvironmentType, NetworkType, UpdaterType, PolicyType>& agent,
  RandomReplay<EnvironmentType>& replayMethod,
  size_t& episodes,
  size_t& consecutiveEpisodes,
  const size_t numSteps)
{
  agent.Deterministic() = false;
  std::cout << "Training for " << numSteps << " steps." << std::endl;
  while (agent.TotalSteps() < numSteps) {
    episodeReturn_ = 0;
    // This one is no longer required since our environment rest automatically
    // env.reset();
    do {
      agent.State().Data() = quadrotor.current_state().Data();

      // Here we need to execute action, swarm_.one..etc
      //  agent.SelectAction();
      //  arma::mat action = { double(agent.Action().action[0] * 2) };

      env.step(action);
      ContinuousActionEnv::State nextState; // replace it with an arma matrix
      nextState.Data() = quadrotor.current_state().Data();
      // 1) we need to store the executed action in agent.Action()
      // Thus we need to execute a lamda function here in order to 
      // recover the values of the function
      // 2) We need to create a reward function to store it
      // 3) We need to uderstand env.done in order to replicate it
      // 4)

      replayMethod.Store(
        agent.State(), agent.Action(), env.reward, nextState, env.done, 0.99);
      episodeReturn += env.reward;
      agent.TotalSteps()++;

      if (agent.Deterministic() ||
          agent.TotalSteps() < config_.ExplorationSteps())
        continue;

      for (size_t i = 0; i < config_.UpdateInterval(); i++)
        agent.Update();
    }

    while (!env.done);
    returnList_.push_back(episodeReturn);

    episodes += 1;

    if (returnList_.size() > consecutiveEpisodes)
      returnList_.erase(returnList.begin());

    double averageReturn =
      std::accumulate(returnList_.begin(), returnList_.end(), 0.0) /
      returnList_.size();
    if (episodes % 4 == 0) {
      std::cout << "Avg return in last " << returnList_.size()
                << " episodes: " << averageReturn
                << "\\t Episode return: " << episodeReturn_
                << "\\t Total steps: " << agent.TotalSteps() << std::endl;
    }
  }
}

template<typename EnvironmentType,
         typename NetworkType,
         typename UpdaterType,
         typename PolicyType,
         typename ReplayType = RandomReplay<EnvironmentType>>
template<class QuadrotorType>
void
SACPredictor<QuadrotorType>::test()
{



}
