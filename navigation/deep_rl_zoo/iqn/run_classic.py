# Copyright 2022 The Deep RL Zoo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""
From the paper "Implicit Quantile Networks for Distributional Reinforcement
Learning" http://arxiv.org/abs/1806.06923.
"""
from absl import app
from absl import flags
from absl import logging
import numpy as np
import torch

# pylint: disable=import-error
from deep_rl_zoo.networks.value import IqnMlpNet
from deep_rl_zoo.iqn import agent
from deep_rl_zoo.checkpoint import PyTorchCheckpoint
from deep_rl_zoo.schedule import LinearSchedule
from deep_rl_zoo import main_loop
from deep_rl_zoo import gym_env
from deep_rl_zoo import greedy_actors
from deep_rl_zoo import replay as replay_lib


FLAGS = flags.FLAGS
flags.DEFINE_string(
    'environment_name',
    'CartPole-v1',
    'Classic control tasks name like CartPole-v1, LunarLander-v2, MountainCar-v0, Acrobot-v1.',
)
flags.DEFINE_integer('replay_capacity', 100000, 'Maximum replay size.')
flags.DEFINE_integer('min_replay_size', 10000, 'Minimum replay size before learning starts.')
flags.DEFINE_integer('batch_size', 64, 'Sample batch size when updating the neural network.')
flags.DEFINE_bool('clip_grad', False, 'Clip gradients, default off.')
flags.DEFINE_float('max_grad_norm', 0.5, 'Max gradients norm when do gradients clip.')
flags.DEFINE_float('exploration_epsilon_begin_value', 1.0, 'Begin value of the exploration rate in e-greedy policy.')
flags.DEFINE_float('exploration_epsilon_end_value', 0.05, 'End (decayed) value of the exploration rate in e-greedy policy.')
flags.DEFINE_float('exploration_epsilon_decay_step', 100000, 'Total steps to decay value of the exploration rate.')
flags.DEFINE_float('eval_exploration_epsilon', 0.01, 'Fixed exploration rate in e-greedy policy for evaluation.')

flags.DEFINE_float('priority_exponent', 0.6, 'Priority exponent used in prioritized replay.')
flags.DEFINE_float('importance_sampling_exponent_begin_value', 0.4, 'Importance sampling exponent begin value.')
flags.DEFINE_float('importance_sampling_exponent_end_value', 1.0, 'Importance sampling exponent end value after decay.')
flags.DEFINE_bool('normalize_weights', True, 'Normalize sampling weights in prioritized replay.')

flags.DEFINE_float('huber_param', 1.0, 'Huber loss parameter.')
flags.DEFINE_integer('tau_latent_dim', 64, 'Embeding layer dimenstion.')
flags.DEFINE_integer('tau_samples_policy', 64, 'Number of samples to pull from the network when choose actions.')

flags.DEFINE_float('learning_rate', 0.0001, 'Learning rate.')
flags.DEFINE_float('discount', 0.99, 'Discount rate.')
flags.DEFINE_integer('num_iterations', 2, 'Number of iterations to run.')
flags.DEFINE_integer('num_train_steps', int(5e5), 'Number of training env steps to run per iteration.')
flags.DEFINE_integer('num_eval_steps', int(2e4), 'Number of evaluation env steps to run per iteration.')
flags.DEFINE_integer('learn_interval', 2, 'The frequency (measured in agent steps) to update parameters.')
flags.DEFINE_integer(
    'target_net_update_interval',
    100,
    'The frequency (measured in number of Q network parameter updates) to update target networks.',
)
flags.DEFINE_integer('seed', 1, 'Runtime seed.')
flags.DEFINE_bool('use_tensorboard', True, 'Use Tensorboard to monitor statistics, default on.')
flags.DEFINE_bool('actors_on_gpu', True, 'Run actors on GPU, default on.')
flags.DEFINE_integer(
    'debug_screenshots_interval',
    0,
    'Take screenshots every N episodes and log to Tensorboard, default 0 no screenshots.',
)
flags.DEFINE_string('tag', '', 'Add tag to Tensorboard log file.')
flags.DEFINE_string('results_csv_path', './logs/iqn_classic_results.csv', 'Path for CSV log file.')
flags.DEFINE_string('checkpoint_dir', './checkpoints', 'Path for checkpoint directory.')


def main(argv):
    """Trains IQN agent on classic control tasks."""
    del argv
    runtime_device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    logging.info(f'Runs IQN agent on {runtime_device}')
    np.random.seed(FLAGS.seed)
    torch.manual_seed(FLAGS.seed)
    if torch.backends.cudnn.enabled:
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True

    random_state = np.random.RandomState(FLAGS.seed)  # pylint: disable=no-member

    # Create environment.
    def environment_builder():
        return gym_env.create_classic_environment(
            env_name=FLAGS.environment_name,
            seed=random_state.randint(1, 2**10),
        )

    train_env = environment_builder()
    eval_env = environment_builder()

    logging.info('Environment: %s', FLAGS.environment_name)
    logging.info('Action spec: %s', train_env.action_space.n)
    logging.info('Observation spec: %s', train_env.observation_space.shape[0])

    state_dim = train_env.observation_space.shape[0]
    action_dim = train_env.action_space.n

    network = IqnMlpNet(state_dim=state_dim, action_dim=action_dim, latent_dim=FLAGS.tau_latent_dim)
    optimizer = torch.optim.Adam(network.parameters(), lr=FLAGS.learning_rate)

    # Test network input and output
    obs = train_env.reset()
    s = torch.from_numpy(obs[None, ...]).float()
    network_output = network(s, num_taus=1)  # only one sample
    q_dist = network_output.q_dist
    q_values = network_output.q_values
    taus = network_output.taus
    assert q_dist.shape == (1, 1, action_dim)
    assert q_values.shape == (1, action_dim)
    assert taus.shape == (1, 1)

    # Create e-greedy exploration epsilon schedule
    exploration_epsilon_schedule = LinearSchedule(
        begin_t=int(FLAGS.min_replay_size),
        decay_steps=int(FLAGS.exploration_epsilon_decay_step),
        begin_value=FLAGS.exploration_epsilon_begin_value,
        end_value=FLAGS.exploration_epsilon_end_value,
    )

    # Create prioritized transition replay
    # Note the t in the replay is not exactly aligned with the agent t.
    importance_sampling_exponent_schedule = LinearSchedule(
        begin_t=int(FLAGS.min_replay_size),
        end_t=(FLAGS.num_iterations * int(FLAGS.num_train_steps)),
        begin_value=FLAGS.importance_sampling_exponent_begin_value,
        end_value=FLAGS.importance_sampling_exponent_end_value,
    )
    replay = replay_lib.PrioritizedReplay(
        capacity=FLAGS.replay_capacity,
        structure=replay_lib.TransitionStructure,
        priority_exponent=FLAGS.priority_exponent,
        importance_sampling_exponent=importance_sampling_exponent_schedule,
        normalize_weights=FLAGS.normalize_weights,
        random_state=random_state,
    )

    # Create Iqn agent instance
    train_agent = agent.Iqn(
        network=network,
        optimizer=optimizer,
        transition_accumulator=replay_lib.TransitionAccumulator(),
        replay=replay,
        exploration_epsilon=exploration_epsilon_schedule,
        batch_size=FLAGS.batch_size,
        min_replay_size=FLAGS.min_replay_size,
        learn_interval=FLAGS.learn_interval,
        target_net_update_interval=FLAGS.target_net_update_interval,
        discount=FLAGS.discount,
        clip_grad=FLAGS.clip_grad,
        max_grad_norm=FLAGS.max_grad_norm,
        action_dim=action_dim,
        huber_param=FLAGS.huber_param,
        tau_samples_policy=FLAGS.tau_samples_policy,
        random_state=random_state,
        device=runtime_device,
    )

    # Create evaluation agent instance
    eval_agent = greedy_actors.IqnEpsilonGreedyActor(
        network=network,
        exploration_epsilon=FLAGS.eval_exploration_epsilon,
        random_state=random_state,
        device=runtime_device,
        tau_samples=FLAGS.tau_samples_policy,
    )

    # Setup checkpoint.
    checkpoint = PyTorchCheckpoint(environment_name=FLAGS.environment_name, agent_name='IQN', save_dir=FLAGS.checkpoint_dir)
    checkpoint.register_pair(('network', network))

    # Run the training and evaluation for N iterations.
    main_loop.run_single_thread_training_iterations(
        num_iterations=FLAGS.num_iterations,
        num_train_steps=FLAGS.num_train_steps,
        num_eval_steps=FLAGS.num_eval_steps,
        train_agent=train_agent,
        train_env=train_env,
        eval_agent=eval_agent,
        eval_env=eval_env,
        checkpoint=checkpoint,
        csv_file=FLAGS.results_csv_path,
        use_tensorboard=FLAGS.use_tensorboard,
        tag=FLAGS.tag,
        debug_screenshots_interval=FLAGS.debug_screenshots_interval,
    )


if __name__ == '__main__':
    app.run(main)
