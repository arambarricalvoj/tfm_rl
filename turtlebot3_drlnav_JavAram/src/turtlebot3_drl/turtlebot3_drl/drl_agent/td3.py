import numpy as np
import copy

import torch
import torch.nn.functional as F
import torch.nn as nn

from ..common.settings import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY
from ..common.ounoise import OUNoise

from .off_policy_agent import OffPolicyAgent, Network

LINEAR = 0
ANGULAR = 1
NUM_SCAN_SAMPLES = 500

# Map sizes (as discussed: 24x24 each)
PROB_H = 24
PROB_W = 24
PROB_N = PROB_H * PROB_W
LEM_H = 24
LEM_W = 24
LEM_N = LEM_H * LEM_W

# Reference for network structure: https://arxiv.org/pdf/2102.10711.pdf
# https://github.com/hanlinniu/turtlebot3_ddpg_collision_avoidance/blob/main/turtlebot_ddpg/scripts/original_ddpg/ddpg_network_turtlebot3_original_ddpg.py
# https://github.com/djbyrne/TD3


class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)

        # --- common parameters ---
        out_dimension = 20  # output dimension per branch

        # compute scalar size (everything that's not scan/prob/lem)
        # state layout expected: [ scan(NUM_SCAN_SAMPLES), scalars (actions+yaw etc) , prob_flat(PROB_N), lem_flat(LEM_N) ]
        self.state_size = state_size
        self.action_size = action_size
        self._num_scan = NUM_SCAN_SAMPLES
        self._prob_n = PROB_N
        self._lem_n = LEM_N
        self.scalar_size = state_size - NUM_SCAN_SAMPLES - PROB_N - LEM_N
        if self.scalar_size < 0:
            raise ValueError("state_size too small for configured NUM_SCAN_SAMPLES/PROB_N/LEM_N")

        # --- LASER CNN (1D) ---
        self.cnn_laser = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=7, stride=3),
            nn.ReLU(),
            nn.Conv1d(16, 1, kernel_size=6, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveMaxPool1d(out_dimension),
            nn.Flatten()
        )

        # --- PROB MAP CNN (2D) ---
        # reduce spatially and map to out_dimension
        self.cnn_prob = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(8, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveMaxPool2d((4, 5)),  # example reduction -> 4x5
            nn.Flatten(),
            nn.Linear(16 * 4 * 5, out_dimension),
            nn.ReLU()
        )

        # --- LEM MAP CNN (2D) ---
        self.cnn_lem = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(8, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveMaxPool2d((4, 5)),
            nn.Flatten(),
            nn.Linear(16 * 4 * 5, out_dimension),
            nn.ReLU()
        )

        # --- CONCATENATED FCN---
        # final input dim = laser_out + prob_out + lem_out + scalar_features
        fc_in = out_dimension + out_dimension + out_dimension + self.scalar_size
        self.fa1 = nn.Linear(fc_in, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)

        self.apply(super().init_weights)

        # store out_dimension for forward
        self._out_dim = out_dimension

    def forward(self, states, visualize=False):
        # If no batch we add a batch dimension
        single_dim = False
        if states.dim() == 1:
            states = states.unsqueeze(0)
            single_dim = True

        b = states.shape[0]

        # --- reconstruct inputs from flat state vector ---
        # scan
        scan = states[:, :self._num_scan].unsqueeze(1)  # [B, 1, NUM_SCAN_SAMPLES]

        # scalars (actions + yaw etc)
        s0 = self._num_scan
        s1 = s0 + self.scalar_size
        scalars = states[:, s0:s1]  # [B, scalar_size]

        # prob map flat -> reshape to (B,1,PROB_H,PROB_W)
        p0 = s1
        p1 = p0 + self._prob_n
        prob_flat = states[:, p0:p1]
        prob_map = prob_flat.view(b, 1, PROB_H, PROB_W)

        # lem map flat -> reshape
        l0 = p1
        l1 = l0 + self._lem_n
        lem_flat = states[:, l0:l1]
        lem_map = lem_flat.view(b, 1, LEM_H, LEM_W)

        # --- pass through branches ---
        x_laser = self.cnn_laser(scan)        # (B, out_dimension)
        x_prob = self.cnn_prob(prob_map)      # (B, out_dimension)
        x_lem = self.cnn_lem(lem_map)         # (B, out_dimension)

        # concat all features
        x_combined = torch.cat([x_laser, x_prob, x_lem, scalars], dim=1)

        # --- fully connected ---
        x1 = torch.relu(self.fa1(x_combined))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))

        if single_dim:
            action = action.squeeze(0)

        # optional visualization hook (kept as in original)
        if visualize and self.visual:
            self.visual.update_layers(states, action, [x1, x2], [self.fa1.bias, self.fa2.bias])

        return action


class Critic(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)
        out_dimension = 20

        # compute scalar size consistent with Actor
        self.state_size = state_size
        self._num_scan = NUM_SCAN_SAMPLES
        self._prob_n = PROB_N
        self._lem_n = LEM_N
        self.scalar_size = state_size - NUM_SCAN_SAMPLES - PROB_N - LEM_N
        if self.scalar_size < 0:
            raise ValueError("state_size too small for configured NUM_SCAN_SAMPLES/PROB_N/LEM_N")

        # --- LASER CNN (Twin Extractor) ---
        def get_laser_extractor():
            return nn.Sequential(
                nn.Conv1d(1, 16, kernel_size=7, stride=3),
                nn.ReLU(),
                nn.Conv1d(16, 1, kernel_size=6, stride=1, padding=1),
                nn.ReLU(),
                nn.AdaptiveMaxPool1d(out_dimension),
                nn.Flatten()
            )

        self.cnn_q1_laser = get_laser_extractor()
        self.cnn_q2_laser = get_laser_extractor()

        # --- MAP CNN extractors for Q1 and Q2 (prob + lem) ---
        def get_map_extractor():
            return nn.Sequential(
                nn.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),
                nn.ReLU(),
                nn.Conv2d(8, 16, kernel_size=3, stride=1, padding=1),
                nn.ReLU(),
                nn.AdaptiveMaxPool2d((4, 5)),
                nn.Flatten(),
                nn.Linear(16 * 4 * 5, out_dimension),
                nn.ReLU()
            )

        # Q1 map extractors
        self.cnn_q1_prob = get_map_extractor()
        self.cnn_q1_lem = get_map_extractor()
        # Q2 map extractors
        self.cnn_q2_prob = get_map_extractor()
        self.cnn_q2_lem = get_map_extractor()

        # --- Q1 FCN ---
        # input: laser_out + prob_out + lem_out + scalar_size + action_size
        q_in = out_dimension + out_dimension + out_dimension + self.scalar_size + action_size
        self.l1 = nn.Linear(q_in, hidden_size)
        self.l2 = nn.Linear(hidden_size, hidden_size)
        self.l3 = nn.Linear(hidden_size, 1)

        # --- Q2 FCN ---
        self.l4 = nn.Linear(q_in, hidden_size)
        self.l5 = nn.Linear(hidden_size, hidden_size)
        self.l6 = nn.Linear(hidden_size, 1)

        self.apply(super().init_weights)

        self._out_dim = out_dimension

    def forward(self, states, actions):
        # If no batch we add a batch dimension
        if states.dim() == 1:
            states = states.unsqueeze(0)
        if actions.dim() == 1:
            actions = actions.unsqueeze(0)

        b = states.shape[0]

        # reconstruct inputs
        scan = states[:, :self._num_scan].unsqueeze(1)  # [B,1,NUM_SCAN]
        s0 = self._num_scan
        s1 = s0 + self.scalar_size
        scalars = states[:, s0:s1]  # [B, scalar_size]

        p0 = s1
        p1 = p0 + self._prob_n
        prob_flat = states[:, p0:p1]
        prob_map = prob_flat.view(b, 1, PROB_H, PROB_W)

        l0 = p1
        l1 = l0 + self._lem_n
        lem_flat = states[:, l0:l1]
        lem_map = lem_flat.view(b, 1, LEM_H, LEM_W)

        # --- Q1 branch ---
        x1_laser = self.cnn_q1_laser(scan)
        x1_prob = self.cnn_q1_prob(prob_map)
        x1_lem = self.cnn_q1_lem(lem_map)
        x1 = torch.cat([x1_laser, x1_prob, x1_lem, scalars, actions], dim=1)
        x1 = torch.relu(self.l1(x1))
        x1 = torch.relu(self.l2(x1))
        q1 = self.l3(x1)

        # --- Q2 branch ---
        x2_laser = self.cnn_q2_laser(scan)
        x2_prob = self.cnn_q2_prob(prob_map)
        x2_lem = self.cnn_q2_lem(lem_map)
        x2 = torch.cat([x2_laser, x2_prob, x2_lem, scalars, actions], dim=1)
        x2 = torch.relu(self.l4(x2))
        x2 = torch.relu(self.l5(x2))
        q2 = self.l6(x2)

        return q1, q2

    def Q1_forward(self, states, actions):
        # If no batch we add a batch dimension
        if states.dim() == 1:
            states = states.unsqueeze(0)

        b = states.shape[0]
        scan = states[:, :self._num_scan].unsqueeze(1)
        s0 = self._num_scan
        s1 = s0 + self.scalar_size
        scalars = states[:, s0:s1]

        p0 = s1
        p1 = p0 + self._prob_n
        prob_flat = states[:, p0:p1]
        prob_map = prob_flat.view(b, 1, PROB_H, PROB_W)

        l0 = p1
        l1 = l0 + self._lem_n
        lem_flat = states[:, l0:l1]
        lem_map = lem_flat.view(b, 1, LEM_H, LEM_W)

        x1_laser = self.cnn_q1_laser(scan)
        x1_prob = self.cnn_q1_prob(prob_map)
        x1_lem = self.cnn_q1_lem(lem_map)
        x1 = torch.cat([x1_laser, x1_prob, x1_lem, scalars, actions], dim=1)
        x1 = torch.relu(self.l1(x1))
        x1 = torch.relu(self.l2(x1))
        return self.l3(x1)


class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        # DRL parameters
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, min_sigma=0.1, decay_period=8000000)

        # TD3 parameters
        self.policy_noise = POLICY_NOISE
        self.noise_clip = POLICY_NOISE_CLIP
        self.policy_freq = POLICY_UPDATE_FREQUENCY

        self.last_actor_loss = 0

        self.actor = self.create_network(Actor, 'actor')
        self.actor_target = self.create_network(Actor, 'target_actor')
        self.actor_optimizer = self.create_optimizer(self.actor)

        self.critic = self.create_network(Critic, 'critic')
        self.critic_target = self.create_network(Critic, 'target_critic')
        self.critic_optimizer = self.create_optimizer(self.critic)

        self.hard_update(self.actor_target, self.actor)
        self.hard_update(self.critic_target, self.critic)

    def get_action(self, state, is_training, step, visualize=False):
        state = torch.from_numpy(np.asarray(state, np.float32)).to(self.device)
        action = self.actor(state, visualize)
        if is_training:
            noise = torch.from_numpy(copy.deepcopy(self.noise.get_noise(step))).to(self.device)
            action = torch.clamp(torch.add(action, noise), -1.0, 1.0)
        return action.detach().cpu().data.numpy().tolist()

    def get_action_random(self):
        return [np.clip(np.random.uniform(-1.0, 1.0), -1.0, 1.0)] * self.action_size

    def train(self, state, action, reward, state_next, done):
        noise = (torch.randn_like(action) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
        action_next = (self.actor_target(state_next) + noise).clamp(-1.0, 1.0)
        Q1_next, Q2_next = self.critic_target(state_next, action_next)
        Q_next = torch.min(Q1_next, Q2_next)

        Q_target = reward + (1 - done) * self.discount_factor * Q_next
        Q1, Q2 = self.critic(state, action)

        loss_critic = self.loss_function(Q1, Q_target) + self.loss_function(Q2, Q_target)
        self.critic_optimizer.zero_grad()
        loss_critic.backward()
        nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=2.0, norm_type=2)
        self.critic_optimizer.step()

        if self.iteration % self.policy_freq == 0:
            # optimize actor
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            loss_actor.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
            self.actor_optimizer.step()

            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)
            self.last_actor_loss = loss_actor.mean().detach().cpu()
        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]
