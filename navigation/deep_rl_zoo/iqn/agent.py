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
"""IQN agent class.

From the paper "Implicit Quantile Networks for Distributional Reinforcement
Learning" http://arxiv.org/abs/1806.06923.

This agent combines:

*   Double Q-learning
*   TD n-step bootstrap
*   Prioritized experience replay

IQN network takes two input:
    s_t: the state of the environment at time t
    tau: the uniform random probability

    the network first take the s_t and turn it into an state embedding (for example the conv2d layers)
    then do cos() operation with given taus
    finally merge the state embedding and xxx to the final linear layers of the network

    the output of IQN network is a sample (or batch samples) of the q value distribution

"""

import copy
from typing import Callable, Tuple
import numpy as np
import torch
from torch import nn

# pylint: disable=import-error
import deep_rl_zoo.replay as replay_lib
import deep_rl_zoo.types as types_lib
import deep_rl_zoo.value_learning as rl
from deep_rl_zoo import base

torch.autograd.set_detect_anomaly(True)


class Iqn(types_lib.Agent):
    """IQN agent"""

    def __init__(
        self,
        network: nn.Module,
        optimizer: torch.optim.Optimizer,
        random_state: np.random.RandomState,  # pylint: disable=no-member
        replay: replay_lib.PrioritizedReplay,
        transition_accumulator: replay_lib.TransitionAccumulator,
        exploration_epsilon: Callable[[int], float],
        learn_interval: int,
        target_net_update_interval: int,
        min_replay_size: int,
        batch_size: int,
        action_dim: int,
        huber_param: float,
        tau_samples_policy: int,
        discount: float,
        clip_grad: bool,
        max_grad_norm: float,
        device: torch.device,
    ):
        """
        Args:
            network: the Q network we want to optimize.
            optimizer: the optimizer for Q network.
            random_state: used to sample random actions for e-greedy policy.
            replay: prioritized experience replay.
            transition_accumulator: external helper class to build n-step transition.
            exploration_epsilon: external schedule of e in e-greedy exploration rate.
            learn_interval: the frequency (measured in agent steps) to do learning.
            target_net_update_interval: the frequency (measured in number of online Q network parameter updates)
                 to Update target network parameters.
            min_replay_size: Minimum replay size before start to do learning.
            batch_size: sample batch size.
            action_dim: number of valid actions in the environment.
            huber_param: parameter k for huber loss.
            tau_samples_policy: number of samples to 'pull' from the network for e-greedy policy.
            discount: gamma discount for future rewards.
            clip_grad: if True, clip gradients norm.
            max_grad_norm: maximum gradient norm for clip grad, only works if clip_grad is True.
            device: PyTorch runtime device.
        """
        if not 1 <= learn_interval:
            raise ValueError(f'Expect learn_interval to be positive integer, got {learn_interval}')
        if not 1 <= target_net_update_interval:
            raise ValueError(f'Expect target_net_update_interval to be positive integer, got {target_net_update_interval}')
        if not 1 <= min_replay_size:
            raise ValueError(f'Expect min_replay_size to be positive integer, got {min_replay_size}')
        if not 1 <= batch_size <= 512:
            raise ValueError(f'Expect batch_size [1, 512], got {batch_size}')
        if not batch_size <= min_replay_size <= replay.capacity:
            raise ValueError(f'Expect min_replay_size >= {batch_size} and <= {replay.capacity} and, got {min_replay_size}')
        if not 0 < action_dim:
            raise ValueError(f'Expect action_dim to be positive integer, got {action_dim}')
        if not 0.0 <= discount <= 1.0:
            raise ValueError(f'Expect discount [0.0, 1.0], got {discount}')
        if not 0.0 <= huber_param:
            raise ValueError(f'Expect huber_param to be greater than or equal to 0.0, got {huber_param}')
        if not 0 < tau_samples_policy:
            raise ValueError(f'Expect tau_samples_policy to be positive integer, got {tau_samples_policy}')

        self.agent_name = 'IQN'
        self._device = device
        self._random_state = random_state
        self._action_dim = action_dim

        # Online Q network
        self._online_network = network.to(device=self._device)
        self._optimizer = optimizer

        # Lazy way to create target Q network
        self._target_network = copy.deepcopy(self._online_network).to(device=self._device)
        # Disable autograd for target network
        for p in self._target_network.parameters():
            p.requires_grad = False

        # Experience replay parameters
        self._transition_accumulator = transition_accumulator
        self._batch_size = batch_size
        self._replay = replay
        self._max_seen_priority = 1.0

        # Learning related parameters
        self._discount = discount
        self._exploration_epsilon = exploration_epsilon
        self._min_replay_size = min_replay_size
        self._learn_interval = learn_interval
        self._target_net_update_interval = target_net_update_interval
        self._clip_grad = clip_grad
        self._max_grad_norm = max_grad_norm

        # IQN parameters
        self._huber_param = huber_param
        self._tau_policy = tau_samples_policy

        # Counters and stats
        self._step_t = -1
        self._update_t = 0
        self._target_update_t = 0
        self._loss_t = np.nan

    def step(self, timestep: types_lib.TimeStep) -> types_lib.Action:
        """Given current timestep, do a action selection and a series of learn related activities"""
        self._step_t += 1

        a_t = self.act(timestep)

        # Try build transition and add into replay
        for transition in self._transition_accumulator.step(timestep, a_t):
            self._replay.add(transition, priority=self._max_seen_priority)

        # Return if replay is ready
        if self._replay.size < self._min_replay_size:
            return a_t

        # Start to learn
        if self._step_t % self._learn_interval == 0:
            self._learn()

        return a_t

    def reset(self):
        """This method should be called at the beginning of every episode."""
        self._transition_accumulator.reset()

    def act(self, timestep: types_lib.TimeStep) -> types_lib.Action:
        'Given timestep, return an action.'
        a_t = self._choose_action(timestep, self.exploration_epsilon)
        return a_t

    @torch.no_grad()
    def _choose_action(self, timestep: types_lib.TimeStep, epsilon: float) -> int:
        """
        Choose action by following the e-greedy policy with respect to Q values
        Args:
            timestep: the current timestep from env
            epsilon: the e in e-greedy exploration
        Returns:
            a_t: the action to take at s_t
        """
        if self._random_state.rand() <= epsilon:
            # randint() return random integers from low (inclusive) to high (exclusive).
            a_t = self._random_state.randint(0, self._action_dim)
            return a_t

        s_t = torch.from_numpy(timestep.observation[None, ...]).to(device=self._device, dtype=torch.float32)
        q_values = self._online_network(s_t, self._tau_policy).q_values
        a_t = torch.argmax(q_values, dim=-1)
        return a_t.cpu().item()

    def _learn(self):
        transitions, indices, weights = self._replay.sample(self._batch_size)
        priorities = self._update(transitions, weights)

        # Update target network parameters
        if self._update_t > 1 and self._update_t % self._target_net_update_interval == 0:
            self._update_target_network()

        if priorities.shape != (self._batch_size,):
            raise RuntimeError(f'Expect priorities has shape ({self._batch_size},), got {priorities.shape}')
        priorities = np.clip(np.abs(priorities), 0.0, 100.0)  # np.abs(priorities)
        self._max_seen_priority = np.max([self._max_seen_priority, np.max(priorities)])
        self._replay.update_priorities(indices, priorities)

    def _update(self, transitions: replay_lib.Transition, weights: np.ndarray) -> np.ndarray:
        weights = torch.from_numpy(weights).to(device=self._device, dtype=torch.float32)  # [batch_size]
        base.assert_rank_and_dtype(weights, 1, torch.float32)

        self._optimizer.zero_grad()
        losses, priorities = self._calc_loss(transitions)
        # Multiply loss by sampling weights, averaging over batch dimension
        loss = torch.mean(losses * weights.detach())
        loss.backward()

        if self._clip_grad:
            torch.nn.utils.clip_grad_norm_(self._online_network.parameters(), self._max_grad_norm, error_if_nonfinite=True)

        self._optimizer.step()
        self._update_t += 1

        # For logging only.
        self._loss_t = loss.detach().cpu().item()

        return priorities

    def _calc_loss(self, transitions: replay_lib.Transition) -> Tuple[torch.Tensor, np.ndarray]:
        """Calculate loss for a given batch of transitions"""
        s_tm1 = torch.from_numpy(transitions.s_tm1).to(device=self._device, dtype=torch.float32)  # [batch_size, state_shape]
        a_tm1 = torch.from_numpy(transitions.a_tm1).to(device=self._device, dtype=torch.int64)  # [batch_size]
        r_t = torch.from_numpy(transitions.r_t).to(device=self._device, dtype=torch.float32)  # [batch_size]
        s_t = torch.from_numpy(transitions.s_t).to(device=self._device, dtype=torch.float32)  # [batch_size, state_shape]
        done = torch.from_numpy(transitions.done).to(device=self._device, dtype=torch.bool)  # [batch_size]

        # Rank and dtype checks, note states may be images, which is rank 4.
        base.assert_rank_and_dtype(s_tm1, (2, 4), torch.float32)
        base.assert_rank_and_dtype(s_t, (2, 4), torch.float32)
        base.assert_rank_and_dtype(a_tm1, 1, torch.long)
        base.assert_rank_and_dtype(r_t, 1, torch.float32)
        base.assert_rank_and_dtype(done, 1, torch.bool)

        discount_t = (~done).float() * self._discount

        # Compute predicted q values distribution for s_tm1, using online Q network
        network_output = self._online_network(s_tm1, self._tau_policy)
        dist_q_tm1 = network_output.q_dist  # [batch_size, action_dim, num_taus]
        tau_tm1 = network_output.taus  # [batch_size, action_dim, num_taus]

        # Computes predicted q values distribution for s_t, using target Q network and double Q
        with torch.no_grad():
            q_t_selector = self._online_network(s_t, self._tau_policy).q_dist  # [batch_size, action_dim, num_taus]
            target_dist_q_t = self._target_network(s_t, self._tau_policy).q_dist  # [batch_size, action_dim, num_taus]

        # Compute quantile regression loss.
        loss = rl.quantile_double_q_learning(
            dist_q_tm1,
            tau_tm1,
            a_tm1,
            r_t,
            discount_t,
            target_dist_q_t,
            q_t_selector,
            self._huber_param,
        ).loss

        priorities = loss.detach().cpu().numpy()
        return loss, priorities

    def _update_target_network(self):
        """Copy online network parameters to target network."""
        self._target_network.load_state_dict(self._online_network.state_dict())
        self._target_update_t += 1

    @property
    def exploration_epsilon(self):
        """Call external schedule function"""
        return self._exploration_epsilon(self._step_t)

    @property
    def statistics(self):
        """Returns current agent statistics as a dictionary."""
        return {
            # 'learning_rate': self._optimizer.param_groups[0]['lr'],
            'loss': self._loss_t,
            # 'discount': self._discount,
            'updates': self._update_t,
            'target_updates': self._target_update_t,
            'exploration_epsilon': self.exploration_epsilon,
        }
