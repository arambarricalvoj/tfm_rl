from ..common.settings import REWARD_FUNCTION, COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE, SUCCESS, TIMEOUT, RESULTS_NUM
import math

# -------------------------
# Parámetros ajustables
# -------------------------
DISCOVERY_SCALE = 1.0
MAX_CELLS_PER_METER = 40.0
MIN_DIST_FOR_PROX_PEN = 0.22
PROX_PENALTY = -2.0
COLLISION_PENALTY = -20.0
TUMBLE_PENALTY = -20.0
TIMEOUT_PENALTY = -5.0
SPIN_PENALTY_SCALE = 0.02
REVISIT_DECAY = 0.5

# Parámetros por defecto para la combinación (ajustables)
params = {
    'alpha': 1.0,        # peso de exploración (E_t)
    'beta': 0.2,         # regularizador de avance
    'gamma': -0.8,       # penaliza giros grandes (negativo)
    'delta': 0.6,        # favorece espacio libre (sensor)
    'kappa_E': 50.0,     # escala para normalizar E_t (celdas por paso)
    'lambda_E': 0.3,     # suavizado exponencial para E_t
    'v_max': 0.3,        # velocidad lineal máxima (m/s)
    'omega_max': 1.5,    # velocidad angular máxima (rad/s)
    's_scale': 2.0,      # escala para sensor (m)
    'r_min': -1.0,
    'r_max': 1.0
}

# -------------------------
# Estado persistente (reiniciar en reward_initialize)
# -------------------------
E_filtered = 0.0
goal_dist_initial = 0.0

# -------------------------
# Inicialización por episodio
# -------------------------
def reward_initialize(init_distance_to_goal=None):
    """
    Reinicia estado por episodio. Llamar desde env.reset().
    Si se pasa init_distance_to_goal, se guarda para compatibilidad con get_reward_A.
    """
    global E_filtered, goal_dist_initial
    E_filtered = 0.0
    if init_distance_to_goal is not None:
        try:
            goal_dist_initial = float(init_distance_to_goal)
        except Exception:
            pass

# -------------------------
# Recompensa antigua (navegación) — compatibilidad
# -------------------------
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
    if min_obstacle_dist < MIN_DIST_FOR_PROX_PEN:
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

# -------------------------
# Recompensa de exploración (integrada, devuelve float)
# -------------------------
def get_reward_explore(succeed, action_linear, action_angular, cov_incr, min_obstacle_dist):
    # [-3.14, 0] no es demasiado fuerte
    #r_yaw = float(-1 * abs(goal_angle))  # Penalize error in orientation towards goal

    # [-4, 0] bien
    r_vangular = -1 * (action_angular ** 2)  # Penalize high angular velocities

    # [-1, 1] es suave
    #try:
    #    gd_init = float(goal_dist_initial) if goal_dist_initial != 0 else float(goal_dist)
    #except Exception:
    #    gd_init = float(goal_dist)
    #r_distance = (2 * gd_init) / (gd_init + float(goal_dist)) - 1  # Reward getting closer to the goal

    # [-20, 0] muy fuerte
    if min_obstacle_dist < MIN_DIST_FOR_PROX_PEN:
        r_obstacle = -20.0
    else:
        r_obstacle = 0.0

    # [-2 * (3^2), 0] fuerte
    r_vlinear = -1 * (((0.3 - action_linear) * 10) ** 2)  # Penalize going Velocities different than max robot velocity

    r_cov = 8 * cov_incr

    reward = (r_cov + r_obstacle + r_vangular + r_vlinear - 10) / 1000.0
    if succeed == SUCCESS:
        reward += 2.0
    elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL):
        reward -= 1.0
    return float(reward)

# -------------------------
# Wrapper get_reward: compatibilidad con firma antigua y nueva
# -------------------------
reward_function_internal = None

def get_reward(*args):
    """
    Wrapper flexible:
    - Si REWARD_FUNCTION == 'explore', interpreta la llamada como la firma nueva:
        (succeed, action_linear, action_angular, min_obstacle_dist, n_new_cells, [allow_discovery_reward], ...)
      Requiere al menos 5 argumentos en ese caso.
    - Si REWARD_FUNCTION != 'explore', usa la firma antigua:
        (succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance)
    Devuelve siempre float.
    """
    if reward_function_internal is None:
        raise RuntimeError("No reward function configurada. Revisa REWARD_FUNCTION en settings.")

    # Si la función seleccionada es la de exploración, forzamos la interpretación nueva
    if reward_function_internal.__name__ == 'get_reward_explore':
        if len(args) < 5:
            raise TypeError("Para REWARD_FUNCTION='explore' get_reward requiere al menos 5 argumentos: (succeed, action_linear, action_angular, min_obstacle_dist, n_new_cells).")
        # Rellenar defaults: allow_discovery_reward=True, dist_since_last_reward=0.0, extras=None
        padded = list(args) + [True, 0.0, None]
        return reward_function_internal(*padded[:8])
    else:
        # Firma antigua: validar que hay 6 argumentos
        if len(args) < 6:
            raise TypeError("Para la función de navegación get_reward_A se requieren 6 argumentos: (succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance).")
        return reward_function_internal(*args[:6])

# -------------------------
# Selección de la función interna según REWARD_FUNCTION
# -------------------------
function_name = "get_reward_" + str(REWARD_FUNCTION)
if function_name in globals():
    reward_function_internal = globals()[function_name]
else:
    # fallback: si REWARD_FUNCTION no coincide, intentar 'explore' por defecto, luego 'A'
    if 'get_reward_explore' in globals():
        reward_function_internal = globals()['get_reward_explore']
    elif 'get_reward_A' in globals():
        reward_function_internal = globals()['get_reward_A']
    else:
        raise RuntimeError(f"Error: la función de recompensa {function_name} no existe y no hay fallback disponible.")
