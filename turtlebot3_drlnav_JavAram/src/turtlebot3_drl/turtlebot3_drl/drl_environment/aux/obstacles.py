import os
import xml.etree.ElementTree as ET

NO_GOAL_SPAWN_MARGIN = 0.3 # meters away from any wall

def get_obstacle_coordinates():
    tree = ET.parse(os.getenv('DRLNAV_BASE_PATH') + '/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/inner_walls/model.sdf')
    root = tree.getroot()
    obstacle_coordinates = []

    for wall in root.find('model').findall('link'):
        pose = wall.find('pose').text.split(" ")
        size = wall.find('collision').find('geometry').find('box').find('size').text.split()
        rotation = float(pose[-1])
        pose_x = float(pose[0])
        pose_y = float(pose[1])

        if rotation == 0:
            size_x = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
            size_y = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
        else:
            size_x = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
            size_y = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2

        p1 = [pose_x + size_x / 2, pose_y + size_y / 2]
        p2 = [p1[0], p1[1] - size_y]
        p3 = [p1[0] - size_x, p1[1] - size_y]
        p4 = [p1[0] - size_x, p1[1]]

        obstacle_coordinates.append([p1, p2, p3, p4])

    return obstacle_coordinates
