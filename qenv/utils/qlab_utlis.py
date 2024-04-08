# python imports
import random
import math
# 3rd party imports
import numpy as np
from typing import Union
# quanser imports
from qvl.qlabs import QuanserInteractiveLabs
from hal.utilities.path_planning import RoadMap, RoadMapNode
from qvl.basic_shape import QLabsBasicShape
from qvl.actor import QLabsActor

def connect_to_qlab() -> QuanserInteractiveLabs:
    """
    Establishes a connection to the Quanser Interactive Labs.

    This function initializes the QuanserInteractiveLabs object and opens a connection
    to the local host where the Quanser Interactive Labs are running.

    Returns:
        QuanserInteractiveLabs: An instance of the QuanserInteractiveLabs class with an open connection.
    """
    qlabs = QuanserInteractiveLabs()
    qlabs.open("localhost")
    return qlabs

def plot_waypoints(qlabs: QuanserInteractiveLabs, sequence: np.ndarray) -> None:
    rotation:list = [0, 0, 0]
    scale:list = [0.01, 0.01, 0.01]
    basic_shape: QLabsBasicShape = QLabsBasicShape(qlabs=qlabs, verbose=False)
    for i in range(len(sequence[0])):
        location = [sequence[0][i], sequence[1][i], 1]
        basic_shape.spawn_id(i, location=location, rotation=rotation, scale=scale, configuration=1, waitForConfirmation=False)

def get_random_deviate(boundry: float) -> float:
    return random.uniform(-boundry, boundry)

def cal_waypoint_angles(waypoint_sequence: np.ndarray, i) -> float:
    waypoint_i = (waypoint_sequence[i][0], waypoint_sequence[i][1])
    waypoint_j = (waypoint_sequence[i+1][0], waypoint_sequence[i+1][1])
    delta_x = waypoint_j[0] - waypoint_i[0]
    delta_y = waypoint_j[1] - waypoint_i[1]
    # print(delta_x, delta_y)
    if delta_x < 0 and delta_y == 0:
        return math.pi # up to bottom
    elif delta_x == 0 and delta_y > 0:
        return math.pi / 2 # right to left
    elif delta_x < 0 and delta_y > 0:
        return math.atan(delta_y / delta_x) + math.pi # right bottom
    elif delta_x > 0 and delta_y > 0:
        return math.atan(delta_y / delta_x) # left bottom
    elif delta_x > 0 and delta_y == 0:
        return 0 # bottom to up
    elif delta_x > 0 and delta_y < 0:
        return math.atan(delta_y / delta_x) # left top
    elif delta_x == 0 and delta_y < 0:
        return 3 * math.pi / 2 # left to right
    else:
        return math.atan(delta_y / delta_x) + math.pi # right top

def get_waypoint_angles(waypoint_sequence: np.ndarray) -> list:
    waypoint_angles: list = []
    for i in range(len(waypoint_sequence)):
        if i != len(waypoint_sequence) - 1:
            waypoint_angles.append(cal_waypoint_angles(waypoint_sequence, i))
        else:
            waypoint_angles.append(waypoint_angles[i-1])
    return waypoint_angles

def get_deviate_state(position: Union[list, np.ndarray]) -> list:
    # get random deviate
    deviate: float = get_random_deviate(0.1)
    orientation_deviate: float = get_random_deviate(0.15)
    # deviated position
    x_position: float = position[0] + deviate * math.cos(position[2])
    y_position: float = position[1] + deviate * math.sin(position[2])
    orientation: float = position[2] + orientation_deviate

    return [x_position, y_position, orientation]

def spawn_on_node(roadmap: RoadMap, actor: QLabsActor, node_id: int = 2, add_deviate: bool = False) -> None:
    if roadmap is None or actor is None:
        raise Exception("Parameters cannot be None")

    node: RoadMapNode = roadmap.nodes[node_id]
    # node position
    x_position: float = node.pose[0]
    y_position: float = node.pose[1]
    orientation: float = node.pose[2]
    # add random deviate
    if add_deviate:
        deviated_position: list = get_deviate_state([x_position, y_position, orientation])
        x_position = deviated_position[0]
        y_position = deviated_position[1]
        orientation = deviated_position[2]
    # spawn actor on the road map
    actor.spawn_id_degrees(
        actorNumber=0,
        location=[x_position, y_position, 0],
        rotation=[0, 0, orientation],
        scale=[.1, .1, .1],
        configuration=0,
        waitForConfirmation=True
    )

def spawn_on_waypoints(waypoint_sequence: np.ndarray, waypoint_angles: list, actor: QLabsActor, add_deviate: bool = True) -> None:
    if waypoint_sequence is None or actor is None:
        raise Exception("parameters cannot be None")

    waypoint_num: int = random.randint(0, len(waypoint_sequence) - 1)
    x_position: float = waypoint_sequence[waypoint_num][0]
    y_position: float = waypoint_sequence[waypoint_num][1]
    orientation: float = waypoint_angles[waypoint_num]
    # add random deviate
    if add_deviate:
        deviated_position: list = get_deviate_state([x_position, y_position, orientation])
        x_position = deviated_position[0]
        y_position = deviated_position[1]
        orientation = deviated_position[2]
    # spawn actor on the road map
    print(x_position, y_position, orientation)
    actor.spawn_id_degrees(
        actorNumber=0,
        location=[x_position, y_position, 0],
        rotation=[0, 0, orientation],
        scale=[.1, .1, .1],
        configuration=0,
        waitForConfirmation=True
    )
