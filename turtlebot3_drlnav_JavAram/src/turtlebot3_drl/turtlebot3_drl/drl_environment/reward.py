from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM
import math

goal_dist_initial = 0

reward_function_internal = None

def get_reward(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance):
    return reward_function_internal(succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)

def get_reward_A(succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist):
        # [-3.14, 0]
        r_yaw = float(-1 * abs(goal_angle)) # Penalize error in orientation towards goal

        # [-4, 0]
        r_vangular = -1 * (action_angular**2) # Penalize high angular velocities

        # [-1, 1]
        r_distance = (2 * float(goal_dist_initial)) / (float(goal_dist_initial) + float(goal_dist)) - 1 # Reward getting closer to the goal

        # [-20, 0]
        if min_obstacle_dist < 0.22: # Penalize being too close to obstacles
            r_obstacle = -20
        else:
            r_obstacle = 0

        # [-2 * (3^2), 0]
        r_vlinear = -1 * (((0.3 - action_linear) * 10) ** 2) # Penalize going Velocities different than max robot velocity

        reward = (r_yaw + r_distance + r_obstacle + r_vangular+ r_vlinear - 10)/1000 # Added -1 as a time penalty
        #reward = (r_yaw + r_distance*10 + r_vangular - 10)/10000 # Added -1 as a time penalty
        #reward = (r_distance - 1)/1000 # Added -1 as a time penalty
        if succeed == SUCCESS:
            reward += 2.0
        elif succeed == COLLISION_OBSTACLE or succeed == COLLISION_WALL or succeed == TIMEOUT:
            reward -= 1.0
        return float(reward)

def get_reward_exploration(
        succeed,
        action_linear,
        action_angular,
        min_obstacle_dist,
        entropy_prev,
        entropy_current,
        angle_to_entropy_region
    ):

    # Penaliza no mirar hacia la zona de mayor entropía
    r_yaw = -1.0 * abs(angle_to_entropy_region)

    # Recompensa reducir entropía (progreso de exploración)
    r_progress = (entropy_prev - entropy_current)

    #r_forward = action_linear * math.cos(angle_to_entropy_region)

    # Penaliza velocidades angulares altas
    r_vangular = -(action_angular ** 2)

    # Penaliza desviarse de la velocidad lineal deseada
    r_vlinear = -(((0.3 - action_linear) * 10) ** 2)

    # Penaliza estar demasiado cerca de obstáculos
    r_obstacle = -20 if min_obstacle_dist < 0.22 else 0

    # Estructura original: suma + penalización temporal
    reward = (r_yaw + r_progress + r_obstacle + r_vangular + r_vlinear - 10) / 1000

    # Recompensas/penalizaciones por final de episodio
    if succeed == SUCCESS:
        reward += 2.0
    elif succeed in [COLLISION_OBSTACLE, COLLISION_WALL, TIMEOUT]:
        reward -= 1.0

    return float(reward)

# Define your own reward function by defining a new function: 'get_reward_X'
# Replace X with your reward function name and configure it in settings.py

def reward_initalize(init_distance_to_goal):
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal

function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals()[function_name]
if reward_function_internal == None:
    quit(f"Error: reward function {function_name} does not exist")
