import os
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
from turtlebot3_drl.drl_environment.reward import SUCCESS
from .settings import GRAPH_DRAW_INTERVAL, GRAPH_AVERAGE_REWARD
from matplotlib.ticker import MaxNLocator

matplotlib.use('TkAgg')
class Graph():
    def __init__(self):
        plt.show()

        self.session_dir = ""
        self.legend_labels = ['Unknown', 'Success', 'Collision Wall', 'Collision Dynamic', 'Timeout', 'Tumble']
        self.legend_colors = ['b', 'g', 'r', 'c', 'm', 'y']

        self.outcome_histories = []

        self.global_steps = 0
        self.data_outcome_history = []
        self.data_rewards = []
        self.data_loss_critic = []
        self.data_loss_actor = []
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]

        self.fig, self.ax = plt.subplots(2, 2)
        self.fig.set_size_inches(18.5, 10.5)

        self.data_exploration = []
        self.fig_exploration, self.ax_exploration = plt.subplots()
        self.fig_exploration.set_size_inches(9.25, 5.25)

        self.ax_exploration.set_title('environment exploration (%)')
        self.ax_exploration.xaxis.set_major_locator(MaxNLocator(integer=True))

        titles = ['outcomes', 'avg critic loss over episode', 'avg actor loss over episode', 'avg reward over 10 episodes']
        for i in range(4):
            ax = self.ax[int(i/2)][int(i%2!=0)]
            ax.set_title(titles[i])
            ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.legend_set = False

    def set_graphdata(self, graphdata, episode):
        self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor = [graphdata[i] for i in range(len(self.graphdata))]
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]
        self.draw_plots(episode)
        return self.global_steps

    def update_data(self, step, global_steps, outcome, reward_sum, loss_critic_sum, loss_actor_sum, exploration_pct):
        self.global_steps = global_steps
        self.data_outcome_history.append(outcome)
        self.data_rewards.append(reward_sum)
        self.data_loss_critic.append(loss_critic_sum / step)
        self.data_loss_actor.append(loss_actor_sum / step)
        self.graphdata = [self.global_steps, self.data_outcome_history, self.data_rewards, self.data_loss_critic, self.data_loss_actor]
        self.data_exploration.append(exploration_pct*100.0)

    def draw_plots(self, episode):
        xaxis = np.array(range(1, episode + 1))

        # Plot outcome history
        for idx in range(len(self.data_outcome_history)):
            if idx == 0:
                self.outcome_histories = [[0],[0],[0],[0],[0],[0]]
                self.outcome_histories[self.data_outcome_history[0]][0] += 1
            else:
                for outcome_history in self.outcome_histories:
                    outcome_history.append(outcome_history[-1])
                self.outcome_histories[self.data_outcome_history[idx]][-1] += 1

        if len(self.data_outcome_history) > 0:
            i = 0
            for outcome_history in self.outcome_histories:
                self.ax[0][0].plot(xaxis, outcome_history, color=self.legend_colors[i], label=self.legend_labels[i])
                i += 1
            if not self.legend_set:
                self.ax[0][0].legend()
                self.legend_set = True

        # Plot critic loss
        y = np.array(self.data_loss_critic)
        self.ax[0][1].plot(xaxis, y)

        # Plot actor loss
        y = np.array(self.data_loss_actor)
        self.ax[1][0].plot(xaxis, y)

        # Plot average reward
        count = int(episode / GRAPH_AVERAGE_REWARD)
        if count > 0:
            xaxis = np.array(range(GRAPH_AVERAGE_REWARD, episode+1, GRAPH_AVERAGE_REWARD))
            averages = list()
            for i in range(count):
                avg_sum = 0
                for j in range(GRAPH_AVERAGE_REWARD):
                    avg_sum += self.data_rewards[i * GRAPH_AVERAGE_REWARD + j]
                averages.append(avg_sum / GRAPH_AVERAGE_REWARD)
            y = np.array(averages)
            self.ax[1][1].plot(xaxis, y)

        plt.draw()
        plt.pause(0.2)
        plt.savefig(os.path.join(self.session_dir, "_figure.png"))
        self.draw_exploration_plot()

    def get_success_count(self):
        suc = self.data_outcome_history[-GRAPH_DRAW_INTERVAL:]
        return suc.count(SUCCESS)

    def get_reward_average(self):
        rew = self.data_rewards[-GRAPH_DRAW_INTERVAL:]
        return sum(rew) / len(rew)

    """def draw_exploration_plot(self):
        self.ax_exploration.clear()

        self.ax_exploration.set_title('environment exploration over episode')
        self.ax_exploration.set_xlabel('episode')
        self.ax_exploration.set_ylabel('exploration (%)')
        self.ax_exploration.xaxis.set_major_locator(MaxNLocator(integer=True))

        if len(self.data_exploration) > 0:
            xaxis = np.array(range(1, len(self.data_exploration) + 1))
            y = np.array(self.data_exploration)

            self.ax_exploration.plot(xaxis, y)

        self.fig_exploration.savefig(
            os.path.join(self.session_dir, "_exploration.png")
        )"""

    def draw_exploration_plot(self):
        self.ax_exploration.clear()

        self.ax_exploration.set_title('environment exploration (%)')
        self.ax_exploration.set_xlabel('episode')
        self.ax_exploration.set_ylabel('exploration (%)')
        self.ax_exploration.xaxis.set_major_locator(MaxNLocator(integer=True))

        n = len(self.data_exploration)

        if n > 0:

            # ============================================================
            # 1) CURVA RAW (episodio a episodio)
            # ============================================================
            x_raw = np.arange(1, n + 1)
            y_raw = np.array(self.data_exploration)

            self.ax_exploration.plot(
                x_raw, y_raw,
                color='lightgray',
                linewidth=1,
                label='raw (episodio a episodio)'
            )

            # ============================================================
            # 2) MEDIA POR BLOQUES (igual que reward)
            # ============================================================
            count = n // GRAPH_AVERAGE_REWARD
            if count > 0:
                x_blocks = np.arange(
                    GRAPH_AVERAGE_REWARD,
                    n + 1,
                    GRAPH_AVERAGE_REWARD
                )

                averages = []
                for i in range(count):
                    block = self.data_exploration[
                        i * GRAPH_AVERAGE_REWARD : (i + 1) * GRAPH_AVERAGE_REWARD
                    ]
                    averages.append(sum(block) / GRAPH_AVERAGE_REWARD)

                y_blocks = np.array(averages)

                self.ax_exploration.plot(
                    x_blocks, y_blocks,
                    color='blue',
                    linewidth=2,
                    label=f'media cada {GRAPH_AVERAGE_REWARD} episodios'
                )

            # ============================================================
            # 3) MEDIA MÓVIL (tendencia continua)
            # ============================================================
            window = GRAPH_AVERAGE_REWARD  # normalmente 10
            if n >= window:
                moving_avg = np.convolve(
                    self.data_exploration,
                    np.ones(window) / window,
                    mode='valid'
                )
                x_moving = np.arange(window, n + 1)

                self.ax_exploration.plot(
                    x_moving, moving_avg,
                    color='orange',
                    linewidth=3,
                    label=f'media móvil ({window})'
                )

            self.ax_exploration.legend()

        # Guardar la figura con las 3 curvas
        self.fig_exploration.savefig(
            os.path.join(self.session_dir, "_exploration.png")
        )

