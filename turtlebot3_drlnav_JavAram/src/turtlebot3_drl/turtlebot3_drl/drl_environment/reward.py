from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM
import math
import numpy as np

goal_dist_initial = 0

reward_function_internal = None

def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance):
    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)

def get_reward_A(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist):
    # [-3.14, 0] no es demasiado fuerte
    r_yaw = float(-1 * abs(goal_angle))  # Penalize error in orientation towards goal

    # [-4, 0] bien
    r_vangular = -1 * (action_angular ** 2)  # Penalize high angular velocities

    # [-1, 1] es suave
    try:
        gd_init = float(goal_dist_initial) if goal_dist_initial != 0 else float(goal_dist)
    except Exception:
        gd_init = float(goal_dist)
    r_distance = (2 * gd_init) / (gd_init + float(goal_dist)) - 1  # Reward getting closer to the goal

    # [-20, 0] muy fuerte
    if min_obstacle_dist < 0.22:
        r_obstacle = -20.0
    else:
        r_obstacle = 0.0

    # [-2 * (3^2), 0] fuerte
    r_vlinear = -1 * (((0.3 - action_linear) * 10) ** 2)  # Penalize going Velocities different than max robot velocity

    reward = (r_yaw + r_distance + r_obstacle + r_vangular + r_vlinear - 10) / 1000.0
    if succeed == SUCCESS:
        reward += 2.0
    elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL, TIMEOUT):
        reward -= 1.0
    return float(reward)

def get_reward_explore(succeed, action_linear, action_angular, min_obstacle_dist, exploration, steps):

    # [-4, 0] bien
    r_vangular = -1.0 * (action_angular ** 2)
    # [-18, 0] fuerte
    r_vlinear = -1.0 * (((0.3 - action_linear) * 10.0) ** 2)

    #r_vlinear = -0.2 * ((0.3 - action_linear) ** 2)
    #r_vangular = -0.1 * (action_angular ** 2)


    # [-20, 0] muy fuerte
    if min_obstacle_dist < 0.22:
        r_obstacle = -20.0
    else:
        r_obstacle = 0.0

    # --- 3) Exploración (adaptado del paper) ---
    rho_prev = float(exploration["previous"])
    rho_curr = float(exploration["current"])
    delta_sq = rho_curr**2 - rho_prev**2

    if delta_sq > 0.0:
        # Paper: clip(10 * (rho_t^2 - rho_{t-1}^2), 0, 1)
        r_exploration = min(10.0 * delta_sq, 1.0) * 100.0
    else:
        # Paper: -0.005 → lo llevamos a tu escala *1000
        r_exploration = -0.5

    r_time = -1.0 

    reward = (r_vangular + r_vlinear + r_obstacle + r_exploration + r_time) / 1000.0
    if succeed == SUCCESS:
        reward += 2.0
    elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE):
        reward -= 5.0
    elif succeed == TIMEOUT:
        reward -= 1.0

    return float(reward)


def calculate_bee_reward(
    # Parámetros generales
    succeed, min_dist, dist_moved, 
    # r1: Completeness
    M_t, M_t_prev, 
    # r2: Exploration
    heading_change, inf_sectors_indices, 
    # r3: Exploitation
    current_pos, path_history,
    # Hiperparámetros (pesos y umbrales)
    alpha=1.0, beta=1.0, gamma=1.0, 
    tau=0.5, rho=0.5, dismin=0.12, Lmin=0.12
):
    """
    Implementación de la recompensa BEE (Grid Completeness, Exploration, Exploitation)
    basada en Zhao & Hwang (2024).
    """
    
    # --- 1) Grid Completeness Reward (r1) [1] ---
    # El paper integra éxito y colisión directamente en r1
    if M_t >= 0.95:
        r1 = 10.0  # rmapdone: Recompensa positiva alta por completar el mapa
    elif min_dist <= Lmin:
        r1 = -10.0 # rcrash: Penalización por colisión inminente (terminal)
    else:
        # Recompensa proporcional a la mejora de la completitud entre pasos
        r1 = M_t - M_t_prev
    
    # --- 2) Exploration Reward (r2) [3-5] ---
    # Se premia si el giro (heading_change) apunta a un sector que era 'Inf'
    # 'heading_change' debe mapearse al índice del LiDAR correspondiente
    if heading_change in inf_sectors_indices:
        r2 = tau * dist_moved
    else:
        r2 = -tau * dist_moved
        
    # --- 3) Exploitation Reward (r3) [6, 7] ---
    # Incentiva alejarse de los puntos previos del trayecto
    is_far_from_path = True
    for p_i in path_history:
        # Cálculo de distancia euclídea para diccionarios {'x':, 'y':}
        dist_to_p_i = math.sqrt(
            (current_pos['x'] - p_i[0])**2 + 
            (current_pos['y'] - p_i[1])**2
        )
        if dist_to_p_i < dismin:
            is_far_from_path = False
            break # En cuanto esté cerca de un punto, ya no cumple
    
    if is_far_from_path:
        r3 = rho * dist_moved
    else:
        r3 = -rho * dist_moved
        
    # --- Recompensa Final (BEE) [2, 8] ---
    # R(st) = α*r1(st) + β*r2(st) + γ*r3(st)
    total_reward = (alpha * r1) + (beta * r2) + (gamma * r3)
    
    # Bonificaciones adicionales si usas estados terminales personalizados
    # Aunque el paper ya usa rmapdone y rcrash, esto añade un margen extra
    if succeed == SUCCESS:
        total_reward += 2.0
    elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE):
        total_reward -= 5.0
        
    return float(total_reward)


# Define your own reward function by defining a new function: 'get_reward_X'
# Replace X with your reward function name and configure it in settings.py

def reward_initialize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")