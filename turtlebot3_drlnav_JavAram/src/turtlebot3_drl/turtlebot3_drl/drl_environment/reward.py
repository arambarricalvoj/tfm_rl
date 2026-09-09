from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM
import math

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

    # [-2, 0] muy fuerte
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
    
    # 1) Control (Quitamos el divisor 1000 y ajustamos coeficientes)
    # Rango esperado: [0, -0.3] -> Visible para el crítico (SNR adecuada)
    r_vangular = -0.3 * (action_angular ** 2)
    r_vlinear = -0.3 * (((0.3 - action_linear) * 10.0) ** 2)

    # 2) Obstáculos (Sigue siendo una restricción fuerte)
    if min_obstacle_dist < 0.22:
        r_obstacle = -2.0 # Reducido de -20 para no "aplastar" el gradiente, pero sigue siendo fuerte
    else:
        r_obstacle = 0.0

    # 3) Exploración (Adaptado)
    rho_prev = 100 * float(exploration["previous"])
    rho_curr = 100 * float(exploration["current"])
    delta_sq = (rho_curr - rho_prev)**2

    if delta_sq > 0.0:
        # Peso de 0.5 para que la ganancia de mapa compita con el control
        r_exploration = min(5.0 * delta_sq, 1.0) * 0.5
    else:
        # Penalización por estancamiento (visible: -0.05)
        r_exploration = -0.05

    # 4) Tiempo (Eficiencia)
    # Tras 300 pasos, la suma acumulada será ~ -0.6 (equilibrado con la meta)
    r_time = -0.002 

    # --- RECOMPENSA TOTAL (SIN DIVISOR GLOBAL) ---
    reward = r_vangular + r_vlinear + r_obstacle + r_exploration + r_time

    # 5) Eventos Terminales (Se mantienen como los "faros" del aprendizaje)
    if succeed == SUCCESS:
        reward += 5.0
    elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE):
        reward -= 5.0 # Consistente con el éxito
    elif succeed == TIMEOUT:
        reward -= 1.0 # Mantenemos la restricción

    return float(reward)


# Define your own reward function by defining a new function: 'get_reward_X'
# Replace X with your reward function name and configure it in settings.py

def reward_initialize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")