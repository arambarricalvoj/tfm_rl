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

REWARD_FUNCTION = "exploration"

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
#  REWARD DE EXPLORACIÓN (versión optimizada)
# ============================================================

def get_reward_exploration(
    succeed,
    action_linear,
    min_obstacle_dist,
    coverage,
    prev_coverage
):

    # --------------------------------------------------------
    # 1. Recompensa continua por cobertura total
    # --------------------------------------------------------
    # Esto da señal en cada paso, no solo cuando cambia el mapa.
    r_total = 20.0 * coverage

    # --------------------------------------------------------
    # 2. Bonus por incremento de cobertura
    # --------------------------------------------------------
    delta = coverage - prev_coverage
    r_delta = 200.0 * delta if delta > 0 else 0.0

    # --------------------------------------------------------
    # 3. Penalización por quedarse quieto
    # --------------------------------------------------------
    r_motion = -0.02 if abs(action_linear) < 0.05 else 0.0

    # --------------------------------------------------------
    # 4. Penalización suave por obstáculos
    # --------------------------------------------------------
    r_obstacle = -0.05 if min_obstacle_dist < 0.25 else 0.0

    # --------------------------------------------------------
    # 5. Eventos terminales
    # --------------------------------------------------------
    if succeed in [COLLISION_OBSTACLE, COLLISION_WALL, TUMBLE]:
        r_terminal = -1.0
    elif succeed == TIMEOUT:
        r_terminal = -0.5
    elif succeed == SUCCESS:
        r_terminal = +5.0
    else:
        r_terminal = 0.0

    # --------------------------------------------------------
    # Reward final
    # --------------------------------------------------------
    return r_total + r_delta + r_motion + r_obstacle + r_terminal


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
