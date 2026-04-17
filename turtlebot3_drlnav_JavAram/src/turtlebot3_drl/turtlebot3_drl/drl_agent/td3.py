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


# ============================================================
#                           ACTOR
# ============================================================

class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)

        out_dimension = 20

        self.state_size = state_size
        self.action_size = action_size
        self._num_scan = NUM_SCAN_SAMPLES
        self._prob_n = PROB_N
        self._lem_n = LEM_N

        # ❌ Antes: scan + scalars + prob + lem
        # self.scalar_size = state_size - NUM_SCAN_SAMPLES - PROB_N - LEM_N

        # ✅ Ahora: scan + scalars + gm
        self.scalar_size = state_size - NUM_SCAN_SAMPLES - PROB_N

        if self.scalar_size < 0:
            raise ValueError("state_size too small")

        # --- LASER CNN ---
        self.cnn_laser = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=7, stride=3),
            nn.ReLU(),
            nn.Conv1d(16, 1, kernel_size=6, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveMaxPool1d(out_dimension),
            nn.Flatten()
        )

        # --- PROB MAP CNN (COMENTADO) ---
        # self.cnn_prob = nn.Sequential(
        #     nn.Conv2d(1, 16, kernel_size=5, stride=2, padding=1),
        #     nn.ReLU(),
        #     nn.Conv2d(16, 1, kernel_size=3, stride=1, padding=1),
        #     nn.ReLU(),
        #     nn.AdaptiveMaxPool2d((4, 5)),
        #     nn.Flatten(),
        #     nn.Linear(4 * 5, out_dimension),
        #     nn.ReLU()
        # )

        # --- LEM MAP CNN (COMENTADO) ---
        # self.cnn_lem = nn.Sequential(
        #     nn.Conv2d(1, 16, kernel_size=7, stride=1, padding=1),
        #     nn.ReLU(),
        #     nn.Conv2d(16, 1, kernel_size=6, stride=1, padding=1),
        #     nn.ReLU(),
        #     nn.AdaptiveMaxPool2d((4, 5)),
        #     nn.Flatten(),
        #     nn.Linear(4 * 5, out_dimension),
        #     nn.ReLU()
        # )

        # --- GLOBAL REDUCED MAP CNN (NUEVO, IGUAL QUE PROB) ---
        self.cnn_gm = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=5, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 1, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.AdaptiveMaxPool2d((4, 5)),
            nn.Flatten(),
            nn.Linear(4 * 5, out_dimension),
            nn.ReLU()
        )

        # ❌ Antes: laser + prob + lem + scalars
        # fc_in = out_dimension + out_dimension + out_dimension + self.scalar_size

        # ✅ Ahora: laser + gm + scalars
        fc_in = out_dimension + out_dimension + self.scalar_size

        self.fa1 = nn.Linear(fc_in, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)

        self.apply(super().init_weights)
        self._out_dim = out_dimension


    def forward(self, states, visualize=False):
        single_dim = False
        if states.dim() == 1:
            states = states.unsqueeze(0)
            single_dim = True

        b = states.shape[0]

        scan = states[:, :self._num_scan].unsqueeze(1)

        s0 = self._num_scan
        s1 = s0 + self.scalar_size
        scalars = states[:, s0:s1]

        # --- PROB (COMENTADO) ---
        # p0 = s1
        # p1 = p0 + self._prob_n
        # prob_flat = states[:, p0:p1]
        # prob_map = prob_flat.view(b, 1, PROB_H, PROB_W)

        # --- LEM (COMENTADO) ---
        # l0 = p1
        # l1 = l0 + self._lem_n
        # lem_flat = states[:, l0:l1]
        # lem_map = lem_flat.view(b, 1, LEM_H, LEM_W)

        # --- GM (NUEVO) ---
        p0 = s1
        p1 = p0 + self._prob_n
        gm_flat = states[:, p0:p1]
        gm_map = gm_flat.view(b, 1, PROB_H, PROB_W)

        x_laser = self.cnn_laser(scan)

        # x_prob = self.cnn_prob(prob_map)
        # x_lem = self.cnn_lem(lem_map)

        x_gm = self.cnn_gm(gm_map)

        # x_combined = torch.cat([x_laser, x_prob, x_lem, scalars], dim=1)
        x_combined = torch.cat([x_laser, x_gm, scalars], dim=1)

        x1 = torch.relu(self.fa1(x_combined))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))

        if single_dim:
            action = action.squeeze(0)

        return action



# ============================================================
#                           CRITIC
# ============================================================

class Critic(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)

        out_dimension = 20

        self.state_size = state_size
        self._num_scan = NUM_SCAN_SAMPLES
        self._prob_n = PROB_N
        self._lem_n = LEM_N

        # self.scalar_size = state_size - NUM_SCAN_SAMPLES - PROB_N - LEM_N
        self.scalar_size = state_size - NUM_SCAN_SAMPLES - PROB_N

        if self.scalar_size < 0:
            raise ValueError("state_size too small")

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

        # self.cnn_q1_prob = get_map_extractor()
        # self.cnn_q1_lem = get_map_extractor()
        # self.cnn_q2_prob = get_map_extractor()
        # self.cnn_q2_lem = get_map_extractor()

        self.cnn_q1_gm = get_map_extractor()
        self.cnn_q2_gm = get_map_extractor()

        # q_in = out_dimension + out_dimension + out_dimension + self.scalar_size + action_size
        q_in = out_dimension + out_dimension + self.scalar_size + action_size

        self.l1 = nn.Linear(q_in, hidden_size)
        self.l2 = nn.Linear(hidden_size, hidden_size)
        self.l3 = nn.Linear(hidden_size, 1)

        self.l4 = nn.Linear(q_in, hidden_size)
        self.l5 = nn.Linear(hidden_size, hidden_size)
        self.l6 = nn.Linear(hidden_size, 1)

        self.apply(super().init_weights)


    def forward(self, states, actions):
        if states.dim() == 1:
            states = states.unsqueeze(0)
        if actions.dim() == 1:
            actions = actions.unsqueeze(0)

        b = states.shape[0]

        scan = states[:, :self._num_scan].unsqueeze(1)

        s0 = self._num_scan
        s1 = s0 + self.scalar_size
        scalars = states[:, s0:s1]

        # p0 = s1
        # p1 = p0 + self._prob_n
        # prob_flat = states[:, p0:p1]
        # prob_map = prob_flat.view(b, 1, PROB_H, PROB_W)

        # l0 = p1
        # l1 = l0 + self._lem_n
        # lem_flat = states[:, l0:l1]
        # lem_map = lem_flat.view(b, 1, LEM_H, LEM_W)

        p0 = s1
        p1 = p0 + self._prob_n
        gm_flat = states[:, p0:p1]
        gm_map = gm_flat.view(b, 1, PROB_H, PROB_W)

        x1_laser = self.cnn_q1_laser(scan)
        # x1_prob = self.cnn_q1_prob(prob_map)
        # x1_lem = self.cnn_q1_lem(lem_map)
        x1_gm = self.cnn_q1_gm(gm_map)

        x1 = torch.cat([x1_laser, x1_gm, scalars, actions], dim=1)

        x1 = torch.relu(self.l1(x1))
        x1 = torch.relu(self.l2(x1))
        q1 = self.l3(x1)

        x2_laser = self.cnn_q2_laser(scan)
        # x2_prob = self.cnn_q2_prob(prob_map)
        # x2_lem = self.cnn_q2_lem(lem_map)
        x2_gm = self.cnn_q2_gm(gm_map)

        x2 = torch.cat([x2_laser, x2_gm, scalars, actions], dim=1)

        x2 = torch.relu(self.l4(x2))
        x2 = torch.relu(self.l5(x2))
        q2 = self.l6(x2)

        return q1, q2



# ============================================================
#                       Q1_forward
# ============================================================

    def Q1_forward(self, states, actions):
        if states.dim() == 1:
            states = states.unsqueeze(0)

        b = states.shape[0]

        scan = states[:, :self._num_scan].unsqueeze(1)

        s0 = self._num_scan
        s1 = s0 + self.scalar_size
        scalars = states[:, s0:s1]

        # p0 = s1
        # p1 = p0 + self._prob_n
        # prob_flat = states[:, p0:p1]
        # prob_map = prob_flat.view(b, 1, PROB_H, PROB_W)

        # l0 = p1
        # l1 = l0 + self._lem_n
        # lem_flat = states[:, l0:l1]
        # lem_map = lem_flat.view(b, 1, LEM_H, LEM_W)

        p0 = s1
        p1 = p0 + self._prob_n
        gm_flat = states[:, p0:p1]
        gm_map = gm_flat.view(b, 1, PROB_H, PROB_W)

        x1_laser = self.cnn_q1_laser(scan)
        # x1_prob = self.cnn_q1_prob(prob_map)
        # x1_lem = self.cnn_q1_lem(lem_map)
        x1_gm = self.cnn_q1_gm(gm_map)

        x1 = torch.cat([x1_laser, x1_gm, scalars, actions], dim=1)

        x1 = torch.relu(self.l1(x1))
        x1 = torch.relu(self.l2(x1))
        return self.l3(x1)



# ============================================================
#                           TD3
# ============================================================

class TD3(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)

        self.noise = OUNoise(action_space=self.action_size, max_sigma=0.1, min_sigma=0.1, decay_period=8000000)

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
            loss_actor = -1 * self.critic.Q1_forward(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            loss_actor.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=2.0, norm_type=2)
            self.actor_optimizer.step()

            self.soft_update(self.actor_target, self.actor, self.tau)
            self.soft_update(self.critic_target, self.critic, self.tau)
            self.last_actor_loss = loss_actor.mean().detach().cpu()

        return [loss_critic.mean().detach().cpu(), self.last_actor_loss]
