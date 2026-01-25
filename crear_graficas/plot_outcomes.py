import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# === CONFIGURACIÓN ===
FICHERO = "ddpg_0_stage_9/_train_stage9_20260124-135002.txt"

# === LECTURA DEL FICHERO ===
# Detecta automáticamente el separador
df = pd.read_csv(FICHERO, sep=None, engine="python")

# Limpia espacios en nombres de columnas
df.columns = df.columns.str.strip()

# Extrae columnas
episodes = df["episode"].values
outcomes = df["success"].values
rewards = df["reward"].values
critic_loss = df["avg_critic_loss"].values
actor_loss = df["avg_actor_loss"].values

# === MAPEO DE OUTCOMES ===
labels = {
    1: "SUCCESS",
    2: "COLL_WALL",
    3: "COLL_OBST",
    4: "TIMEOUT",
    5: "TUMBLE"
}

colors = {
    1: "green",
    2: "red",
    3: "turquoise",
    4: "purple",
    5: "yellow"
}

# === CÁLCULO DE ACUMULADOS ===
acumulados = {k: [] for k in labels.keys()}
conteo = {k: 0 for k in labels.keys()}

for outcome in outcomes:
    conteo[outcome] += 1
    for k in labels.keys():
        acumulados[k].append(conteo[k])

# === GRAFICADO OUTCOMES ===
plt.figure(figsize=(12, 6))
for k in labels.keys():
    plt.plot(
        episodes,
        acumulados[k],
        label=labels[k],
        color=colors[k],
        linewidth=2
    )

plt.xlabel("Episodio")
plt.ylabel("Cantidad acumulada")
plt.title("Evolución de outcomes durante el entrenamiento")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("outcomes_acumulados.png", dpi=200)
plt.show()

# === MEDIA MÓVIL (10 EPISODIOS) ===
window = 10

def moving_average(x, w):
    return np.convolve(x, np.ones(w)/w, mode="valid")

reward_ma = moving_average(rewards, window)
critic_ma = moving_average(critic_loss, window)
actor_ma = moving_average(actor_loss, window)

# === GRAFICADO REWARD ===
plt.figure(figsize=(12, 5))
plt.plot(reward_ma, color="blue", linewidth=2)
plt.xlabel("Episodio")
plt.ylabel("Reward (media móvil 10)")
plt.title("Average reward over 10 episodes")
plt.grid(True)
plt.tight_layout()
plt.savefig("avg_reward_10.png", dpi=200)
plt.show()

# === GRAFICADO CRITIC LOSS ===
plt.figure(figsize=(12, 5))
plt.plot(critic_ma, color="orange", linewidth=2)
plt.xlabel("Episodio")
plt.ylabel("Critic loss (media móvil 10)")
plt.title("Average critic loss over 10 episodes")
plt.grid(True)
plt.tight_layout()
plt.savefig("avg_critic_loss_10.png", dpi=200)
plt.show()

# === GRAFICADO ACTOR LOSS ===
plt.figure(figsize=(12, 5))
plt.plot(actor_ma, color="red", linewidth=2)
plt.xlabel("Episodio")
plt.ylabel("Actor loss (media móvil 10)")
plt.title("Average actor loss over 10 episodes")
plt.grid(True)
plt.tight_layout()
plt.savefig("avg_actor_loss_10.png", dpi=200)
plt.show()
