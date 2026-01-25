import pickle
import glob
import torch
import matplotlib.pyplot as plt

def cargar_pkl(path):
    with open(path, "rb") as f:
        data = pickle.load(f)

    total_steps = data[0]
    rewards = data[1]
    critic_loss_1 = data[2]
    actor_loss = [float(x) for x in data[3]]
    critic_loss_2 = [float(x) for x in data[4]]

    return {
        "total_steps": total_steps,
        "rewards": rewards,
        "critic_loss_1": critic_loss_1,
        "actor_loss": actor_loss,
        "critic_loss_2": critic_loss_2
    }

# Buscar todos los episodios
files = sorted(glob.glob("ddpg_0_stage_9/stage9_episode*.pkl"))

all_rewards = []
all_actor_loss = []
all_critic_loss_1 = []
all_critic_loss_2 = []

for f in files:
    d = cargar_pkl(f)
    all_rewards.extend(d["rewards"])
    all_actor_loss.extend(d["actor_loss"])
    all_critic_loss_1.extend(d["critic_loss_1"])
    all_critic_loss_2.extend(d["critic_loss_2"])

episodes = list(range(1, len(all_rewards) + 1))

# --- Gráfica 1: Recompensa ---
plt.figure()
plt.plot(episodes, all_rewards, label="Reward")
plt.xlabel("Episodio")
plt.ylabel("Recompensa")
plt.title("Recompensa por episodio")
plt.grid()
plt.savefig("reward.png")

# --- Gráfica 2: Media móvil ---
import numpy as np
window = 100
moving_avg = np.convolve(all_rewards, np.ones(window)/window, mode="valid")

plt.figure()
plt.plot(moving_avg, label=f"Media móvil {window}")
plt.xlabel("Episodio")
plt.ylabel("Recompensa media")
plt.title("Media móvil de recompensa")
plt.grid()
plt.savefig("reward_moving_avg.png")

# --- Gráfica 3: Actor loss ---
plt.figure()
plt.plot(all_actor_loss)
plt.xlabel("Episodio")
plt.ylabel("Actor loss")
plt.title("Actor loss")
plt.grid()
plt.savefig("actor_loss.png")

# --- Gráfica 4: Critic loss 1 ---
plt.figure()
plt.plot(all_critic_loss_1)
plt.xlabel("Episodio")
plt.ylabel("Critic loss 1")
plt.title("Critic loss 1")
plt.grid()
plt.savefig("critic_loss_1.png")

# --- Gráfica 5: Critic loss 2 ---
plt.figure()
plt.plot(all_critic_loss_2)
plt.xlabel("Episodio")
plt.ylabel("Critic loss 2")
plt.title("Critic loss 2")
plt.grid()
plt.savefig("critic_loss_2.png")

print("Gráficas generadas correctamente.")
