from ..common.settings import (
    COLLISION_OBSTACLE,
    COLLISION_WALL,
    TUMBLE,
    SUCCESS,
    TIMEOUT,
)

# ============================================================
#  COMPATIBILIDAD CON EL AGENTE
# ============================================================

# El agente espera esta variable
REWARD_FUNCTION = "exploration"

# El agente espera esta variable también
def reward_function_internal(
    succeed,
    action_linear,
    action_angular,
    distance_to_goal,
    goal_angle,
    min_obstacle_dist,
    coverage,
    prev_coverage,
    phase
):
    return get_reward_exploration(
        succeed,
        action_linear,
        min_obstacle_dist,
        coverage,
        prev_coverage
    )


# ============================================================
#  REWARD DE EXPLORACIÓN (versión final)
# ============================================================

def get_reward_exploration(
    succeed,
    action_linear,
    min_obstacle_dist,
    coverage,
    prev_coverage
):
    """
    Reward para exploración pura:
    - Recompensa por aumentar cobertura
    - Penalización por quedarse quieto
    - Penalización por acercarse demasiado a obstáculos
    - Penalizaciones fuertes por colisión / tumble / timeout
    """

    # Penalizaciones fuertes
    if succeed in [COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE]:
        return -1.0

    if succeed == TIMEOUT:
        return -0.5

    if succeed == SUCCESS:
        return 2.0

    # Recompensa por incremento de cobertura
    delta = coverage - prev_coverage
    r_explore = 5.0 * delta

    # Penalización por quedarse quieto
    r_motion = -0.01 if abs(action_linear) < 0.05 else 0.0

    # Penalización por acercarse demasiado a obstáculos
    r_obstacle = -0.2 if min_obstacle_dist < 0.25 else 0.0

    return r_explore + r_motion + r_obstacle


# ============================================================
#  API PRINCIPAL (el agente llama a esto)
# ============================================================

def get_reward(
    succeed,
    action_linear,
    action_angular,
    distance_to_goal,
    goal_angle,
    min_obstacle_dist,
    coverage,
    prev_coverage,
    phase
):
    return reward_function_internal(
        succeed,
        action_linear,
        action_angular,
        distance_to_goal,
        goal_angle,
        min_obstacle_dist,
        coverage,
        prev_coverage,
        phase
    )
