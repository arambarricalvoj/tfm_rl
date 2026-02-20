import math
import random
import numpy

def goal_is_valid(goal_x, goal_y, obstacle_coordinates, arena_length, arena_width):
    if goal_x > arena_length/2 or goal_x < -arena_length/2:
        return False
    if goal_y > arena_width/2 or goal_y < -arena_width/2:
        return False

    for obstacle in obstacle_coordinates:
        if goal_x < obstacle[0][0] and goal_x > obstacle[2][0]:
            if goal_y < obstacle[0][1] and goal_y > obstacle[2][1]:
                return False
    return True


def generate_dynamic_goal_pose(robot_x, robot_y, radius, obstacle_coordinates, arena_length, arena_width):
    tries = 0
    while True:
        ring_position = random.uniform(0, 1)
        origin = radius + numpy.random.normal(0, 0.1)
        goal_offset_x = math.cos(2 * math.pi * ring_position) * origin
        goal_offset_y = math.sin(2 * math.pi * ring_position) * origin

        goal_x = robot_x + goal_offset_x
        goal_y = robot_y + goal_offset_y

        if goal_is_valid(goal_x, goal_y, obstacle_coordinates, arena_length, arena_width):
            return goal_x, goal_y

        tries += 1
        if tries > 100:
            return None
