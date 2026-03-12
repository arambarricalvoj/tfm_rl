import numpy as np
import copy
import math

import torch
import torch.nn.functional as F
import torch.nn as nn

from ..common.settings import POLICY_NOISE, POLICY_NOISE_CLIP, POLICY_UPDATE_FREQUENCY
from ..common.ounoise import OUNoise

from .off_policy_agent import OffPolicyAgent, Network

LINEAR = 0
ANGULAR = 1 
NUM_SCAN_SAMPLES = 500

# Reference for network structure: https://arxiv.org/pdf/2102.10711.pdf
# https://github.com/hanlinniu/turtlebot3_ddpg_collision_avoidance/blob/main/turtlebot_ddpg/scripts/original_ddpg/ddpg_network_turtlebot3_original_ddpg.py
# https://github.com/djbyrne/TD3


class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)

        # --- LASER CNN (sin cambios funcionales) ---
        out_dimension = 20
        self.cnn_extract = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=7, stride=3),
            nn.ReLU(),
            nn.Conv1d(16, 1, kernel_size=6, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveMaxPool1d(out_dimension),
            nn.Flatten()
        )

        # --- GRID 2D CNN (nueva rama) ---
        # procesa la local_grid_6x6 (reshape desde el bloque de 36 valores en el state)
        self.g_dim = 8
        self.cnn_grid = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),  # (B,8,6,6)
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),  # (B,8,1,1)
            nn.Flatten()  # (B,8)
        )

        # --- CONCATENATED FCN ---
        # calcular tamaño de escalares (incluye grid_flat de 36; lo restaremos)
        scalar_size = state_size - NUM_SCAN_SAMPLES  # includes grid_flat + pose + prev_actions + coverage
        scalars_without_grid = scalar_size - 36

        # FC entrada: laser_feat + grid_feat + scalars_without_grid
        self.fa1 = nn.Linear(out_dimension + self.g_dim + scalars_without_grid, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)

        self.apply(super().init_weights)

    def forward(self, states, visualize=False):
        # If no batch we add a batch dimension
        single_dim = False
        if states.dim() == 1:
            states = states.unsqueeze(0)
            single_dim = True

        # Make tensors to be compatible with CNN input and separate laser from scalars         
        scan = states[:, :NUM_SCAN_SAMPLES].unsqueeze(1)  # [Batch, 1, NUM_SCAN_SAMPLES]

        # extract grid_flat and reshape to 2D tensor for cnn_grid
        grid_start = NUM_SCAN_SAMPLES
        grid_end = grid_start + 36
        grid_flat = states[:, grid_start:grid_end]  # [B,36]
        # reshape to [B,1,6,6]
        grid_tensor = grid_flat.view(-1, 1, 6, 6)

        # scalars exclude the 36 grid entries
        scalars = states[:, grid_end:]            # [Batch, scalars_without_grid]

        # La CNN solo recibe laser, la rama cnn_grid procesa la grid local
        x_laser = self.cnn_extract(scan)          # [B, out_dimension]
        x_grid = self.cnn_grid(grid_tensor)       # [B, g_dim]

        # Unión: laser features + grid features + scalars
        x_combined = torch.cat([x_laser, x_grid, scalars], dim=1)
        
        # --- define forward pass here ---
        x1 = torch.relu(self.fa1(x_combined))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))

        if single_dim:
            action = action.squeeze(0)

        # -- define layers to visualize here (optional) ---
        if visualize and self.visual:
            self.visual.update_layers(states, action, [x1, x2], [self.fa1.bias, self.fa2.bias])
        # -- define layers to visualize until here ---
        return action

class Critic(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)
        out_dimension = 20
        scalar_size = state_size - NUM_SCAN_SAMPLES

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

        self.cnn_q1 = get_laser_extractor()
        self.cnn_q2 = get_laser_extractor()

        # --- GRID CNN extractors for Q1 and Q2 (independent) ---
        def get_grid_extractor():
            return nn.Sequential(
                nn.Conv2d(1, 8, kernel_size=3, stride=1, padding=1),
                nn.ReLU(),
                nn.AdaptiveAvgPool2d((1,1)),
                nn.Flatten()
            )

        self.cnn_grid_q1 = get_grid_extractor()
        self.cnn_grid_q2 = get_grid_extractor()

        # scalars without grid
        scalars_without_grid = scalar_size - 36

        # --- Q1 FCN ---
        in_dim = out_dimension + 8 + scalars_without_grid + action_size
        self.l1 = nn.Linear(in_dim, hidden_size)
        self.l2 = nn.Linear(hidden_size, hidden_size)
        self.l3 = nn.Linear(hidden_size, 1)

        # --- Q2 FCN ---
        self.l4 = nn.Linear(in_dim, hidden_size)
        self.l5 = nn.Linear(hidden_size, hidden_size)
        self.l6 = nn.Linear(hidden_size, 1)

        self.apply(super().init_weights)

    def forward(self, states, actions):
        # If no batch we add a batch dimension
        if states.dim() == 1:
            states = states.unsqueeze(0)
        if actions.dim() == 1:
            actions = actions.unsqueeze(0)
            
        # Make tensors to be compatible with CNN input and separate laser from scalars   
        scan = states[:, :NUM_SCAN_SAMPLES].unsqueeze(1)  # [Batch, 1, NUM_SCAN_SAMPLES]

        # extract grid_flat and reshape to 2D tensor for cnn_grid
        grid_start = NUM_SCAN_SAMPLES
        grid_end = grid_start + 36
        grid_flat = states[:, grid_start:grid_end]       # [B,36]
        grid_tensor = grid_flat.view(-1, 1, 6, 6)        # [B,1,6,6]

        scalars = states[:, grid_end:]                   # [B, scalars_without_grid]

        # --- Rama Q1 ---
        x1_laser = self.cnn_q1(scan)
        x1_grid = self.cnn_grid_q1(grid_tensor)
        x1 = torch.cat([x1_laser, x1_grid, scalars, actions], dim=1)
        x1 = torch.relu(self.l1(x1))
        x1 = torch.relu(self.l2(x1))
        q1 = self.l3(x1)

        # --- Rama Q2 ---
        x2_laser = self.cnn_q2(scan)
        x2_grid = self.cnn_grid_q2(grid_tensor)
        x2 = torch.cat([x2_laser, x2_grid, scalars, actions], dim=1)
        x2 = torch.relu(self.l4(x2))
        x2 = torch.relu(self.l5(x2))
        q2 = self.l6(x2)

        return q1, q2

    def Q1_forward(self, states, actions):
        # If no batch we add a batch dimension
        if states.dim() == 1:
            states = states.unsqueeze(0)

        # Make tensors to be compatible with CNN input and separate laser from scalars      
        scan = states[:, :NUM_SCAN_SAMPLES].unsqueeze(1)  # [Batch, 1, NUM_SCAN_SAMPLES]
        grid_start = NUM_SCAN_SAMPLES
        grid_end = grid_start + 36
        grid_flat = states[:, grid_start:grid_end]
        grid_tensor = grid_flat.view(-1, 1, 6, 6)
        scalars = states[:, grid_end:]

        x1_laser = self.cnn_q1(scan)
        x1_grid = self.cnn_grid_q1(grid_tensor)
        x1 = torch.cat([x1_laser, x1_grid, scalars, actions], dim=1)
        x1 = torch.relu(self.l1(x1))
        x1 = torch.relu(self.l2(x1))
        return self.l3(x1)

class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        # DRL parameters
        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, min_sigma=0.1, decay_period=8000000)

        # TD3 parameters
        self.policy_noise   = POLICY_NOISE
        self.noise_clip     = POLICY_NOISE_CLIP
        self.policy_freq    = POLICY_UPDATE_FREQUENCY

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
