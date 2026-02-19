from ..common.settings import (
    REWARD_FUNCTION,
    COLLISION_OBSTACLE,
    COLLISION_WALL,
    TUMBLE,
    SUCCESS,
    TIMEOUT,
    RESULTS_NUM
)
import math

goal_dist_initial = 0
reward_function_internal = None


def get_reward(
    succeed,
    action_linear,
    action_angular,
    distance_to_goal,
    goal_angle,
    min_obstacle_distance
):
    return reward_function_internal(
        succeed,
        action_linear,
        action_angular,
        distance_to_goal,
        goal_angle,
        min_obstacle_distance
    )


# -------------------------------------------------------------------------
# REWARD A (para navegación clásica)
# -------------------------------------------------------------------------
def get_reward_A(
    succeed,
    action_linear,
    action_angular,
    goal_dist,
    goal_angle,
    min_obstacle_dist
):
    r_yaw = -abs(goal_angle)
    r_vangular = -(action_angular ** 2)
    r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1
    r_obstacle = -20 if min_obstacle_dist < 0.22 else 0
    r_vlinear = -(((0.3 - action_linear) * 10) ** 2)

    reward = (
        r_yaw +
        r_distance +
        r_obstacle +
        r_vangular +
        r_vlinear -
        1.0  # time penalty suave
    ) / 50.0

    if succeed == SUCCESS:
        reward += 2.0
    elif succeed in [COLLISION_OBSTACLE, COLLISION_WALL, TIMEOUT]:
        reward -= 1.0

    return float(reward)


# -------------------------------------------------------------------------
# REWARD PARA EXPLORACIÓN BASADA EN GRADIENTE
# -------------------------------------------------------------------------
"""def get_reward_exploration(
    succeed,
    action_linear,
    action_angular,
    min_obstacle_dist,
    entropy_prev,
    entropy_current,
    angle_to_gradient
):
    # [-3.14, 0]
    #r_yaw = -abs(angle_to_gradient)


    r_forward = 3.0 * action_linear * math.cos(angle_to_gradient)
    
    # 1. Progreso de entropía (lo más importante)
    r_progress = 100.0 * (entropy_prev - entropy_current)

    # 2. Avance hacia la dirección del gradiente
    #r_forward = 3.0 * action_linear * math.cos(angle_to_gradient)

    # 3. Penalización por girar demasiado
    # [-4, 0]
    r_vangular = -1.0 * (action_angular ** 2)

    # 4. Penalización por ir lento
    # [-2 * (3^2), 0]
    r_vlinear = -1.0 * ((0.3 - action_linear * 10) ** 2)

    # 5. Obstáculos
    r_obstacle = -20.0 if min_obstacle_dist < 0.22 else 0.0

    # 6. Time penalty suave
    #r_time = -0.01

    reward = (r_forward + r_progress + r_vangular + r_vlinear + r_obstacle -10)/1000

    # Final del episodio
    #if succeed == SUCCESS:
        #reward += 2.0
    if succeed in [COLLISION_OBSTACLE, COLLISION_WALL]:
        reward -= 1.0

    return float(reward)"""

# -------------------------------------------------------------------------
# REWARD PARA EXPLORACIÓN BASADA EN GRADIENTE
# -------------------------------------------------------------------------
def get_reward_exploration(
    succeed,
    action_linear,
    action_angular,
    min_obstacle_dist,
    entropy_prev,
    entropy_current,
    angle_to_gradient
):
    # [-3.14, 0]
    # Ya se incluye en r_forward, por lo que si se añade el robot girará demasiado.
    """
    Si añadeo r_yaw, el robot puede aprender:
    “Girar para alinear la orientación me da más recompensa que avanzar”.

    Y entonces:
    gira mucho, avanza poco, explora lento, se queda “pensando” en orientarse perfectamente,
    se vuelve obsesivo con el ángulo. Consecuencia de un r_yaw demasiado fuerte.
    """
    #r_yaw = -abs(angle_to_gradient)
    
    # 1. Progreso de entropía (lo más importante)
    r_progress = 8.0 * (entropy_prev - entropy_current)

    # 2. Avance hacia la dirección del gradiente
    # [-0.3, 0.3] * k; action linear [0, 0.3]
    r_forward = 3.0 * action_linear * math.cos(angle_to_gradient) - angle_to_gradient

    # 3. Penalización por girar demasiado
    # [-3.61, 0]; action angular [-1.9, 1.9]
    r_vangular = -1.0 * (action_angular ** 2)

    # 4. Penalización por ir lento
    # [-0.09, 0]
    r_vlinear = -1.0 * ((0.3 - action_linear *2) ** 2)

    # 5. Obstáculos
    r_obstacle = -20.0 if min_obstacle_dist < 0.22 else 0.0

    # 6. Time penalty suave
    r_time = -0.001

    reward = (
        r_progress +
        r_forward +
        r_vangular +
        r_vlinear +
        r_obstacle +
        r_time
    )

    # Final del episodio
    if succeed == SUCCESS:
        reward += 2.0
    elif succeed in [COLLISION_OBSTACLE, COLLISION_WALL]:
        reward -= 1.0

    return float(reward)


# -------------------------------------------------------------------------
# Inicialización
# -------------------------------------------------------------------------
def reward_initalize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal


function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal is None:
    quit(f"Error: reward function {function_name} does not exist")
