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

def cal_spawn_orientation(current_pos: list, next_pos: list, is_left_traffic: bool = False) -> float:
    delta_x: float = next_pos[0] - current_pos[0]
    delta_y: float = next_pos[1] - next_pos[1]

    # spawn orientation when right traffic
    orientation: float = 0.0
    if delta_x < 0 and delta_y == 0:
        orientation = math.pi # up to bottom
    elif delta_x == 0 and delta_y > 0:
        orientation = math.pi / 2 # right to left
    elif delta_x < 0 and delta_y > 0:
        orientation = math.atan(delta_y / delta_x) + math.pi # right bottom
    elif delta_x > 0 and delta_y > 0:
        orientation = math.atan(delta_y / delta_x) # left bottom
    elif delta_x > 0 and delta_y == 0:
        orientation = 0.0 # bottom to up
    elif delta_x > 0 and delta_y < 0:
        orientation = math.atan(delta_y / delta_x) # left top
    elif delta_x == 0 and delta_y < 0:
        orientation = 3 * math.pi / 2 # left to right
    else:
        orientation = math.atan(delta_y / delta_x) + math.pi # right top
    # check traffic rule
    if is_left_traffic:
        return orientation + math.pi
    return orientation

def is_between(current_waypoint_state: list, node_state: list, next_waypoint_state: list) -> bool:
    is_between_x: bool = current_waypoint_state[0] < node_state[0] < next_waypoint_state[0]
    is_between_x = is_between_x or (current_waypoint_state[0] > node_state[0] > next_waypoint_state[0])
    is_between_y: bool = current_waypoint_state[1] < node_state[1] < next_waypoint_state[1]
    is_between_y = is_between_y or (current_waypoint_state[1] > node_state[1] > next_waypoint_state[1])
    return is_between_x and is_between_y

def get_waypoints_state(roadmap: RoadMap, node_sequence: list, waypoints: np.ndarray) -> np.ndarray:
    if roadmap is None or node_sequence is None or waypoints is None:
        raise Exception("parameters cannot be None")

    # node states
    node_states: np.ndarray = np.zeros((len(node_sequence), 3))
    for i in range(len(node_sequence)):
        node_id: int = node_sequence[i]
        node: RoadMapNode = roadmap.nodes[node_id]
        position: list = node.pose
        node_states[i] = position
    # merge waypoints and nodes, result format: o....o....o
    node_pointer: int = 0
    waypoint_pointer: int = 0
    merged_pointer: int = 0
    merged_waypoint_states: np.ndarray = np.zeros((len(node_sequence) + len(waypoints), 3))
    while merged_pointer < len(merged_waypoint_states):
        orientation: float = 0.0
        if node_pointer == 0: # start position
            current_pos: list = [node_states[node_pointer][0], node_states[node_pointer][1]]
            next_pos: list = [waypoints[waypoint_pointer][0], waypoints[waypoint_pointer][1]]
            orientation = cal_spawn_orientation(current_pos=current_pos, next_pos=next_pos)
            merged_waypoint_states[merged_pointer] = [current_pos[0], current_pos[1], orientation]
            node_pointer += 1
        elif waypoint_pointer == len(waypoints) - 1: # end position
            next_pos: list = [node_states[node_pointer][0], node_states[node_pointer][1]]
            current_pos: list = [waypoints[waypoint_pointer][0], waypoints[waypoint_pointer][1]]
            orientation = cal_spawn_orientation(current_pos=current_pos, next_pos=next_pos)
            merged_waypoint_states[merged_pointer] = [current_pos[0], current_pos[1], orientation]
            node_pointer += 1
        else: # handle inside
            node_state: list = node_states[node_pointer]
            current_waypoint_state: list = [waypoints[waypoint_pointer][0], waypoints[waypoint_pointer][1]]
            next_waypoint_state: list = [waypoints[waypoint_pointer+1][0], waypoints[waypoint_pointer+1][1]]
            if is_between(current_waypoint_state, node_state, next_waypoint_state): # node between waypoints
                current_to_node: float = cal_spawn_orientation(current_pos=current_waypoint_state, next_pos=node_state)
                node_to_next: float = cal_spawn_orientation(current_pos=node_state, next_pos=next_waypoint_state)
                merged_waypoint_states[merged_pointer] = [current_waypoint_state[0], current_waypoint_state[1], current_to_node]
                merged_waypoint_states[merged_pointer+1] = [node_state[0], node_state[1], node_to_next]
                merged_pointer += 1
                node_pointer += 1
                waypoint_pointer += 1
            else: # node does not between waypoints
                current_to_next: float = cal_spawn_orientation(current_pos=current_waypoint_state, next_pos=next_waypoint_state)
                merged_waypoint_states[merged_pointer] = [current_waypoint_state[0], current_waypoint_state[1], current_to_next]
                waypoint_pointer += 1
        # common step
        merged_pointer += 1

    return merged_waypoint_states

def add_deviate(position: Union[list, np.ndarray]) -> list:
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
        raise Exception("parameters cannot be None")

    node: RoadMapNode = roadmap.nodes[node_id]
    # node position
    x_position: float = node.pose[0]
    y_position: float = node.pose[1]
    orientation: float = node.pose[2]
    # add random deviate
    if add_deviate:
        deviated_position: list = add_deviate([x_position, y_position, orientation])
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

def spawn_on_waypoints(waypoint_states: np.ndarray, actor: QLabsActor, add_deviate: bool = True) -> None:
    if waypoint_states is None or actor is None:
        raise Exception("parameters cannot be None")
    if len(waypoint_states[0]) != 3:
        raise ValueError("Incorrect state format")

    # waypoint_state[i] format: [float, float, float]
    waypoint_num: int = random.randint(0, len(waypoint_states) - 1)
    x_position: float = waypoint_states[waypoint_num][0]
    y_position: float = waypoint_states[waypoint_num][1]
    orientation: float = waypoint_states[waypoint_num][2]
    # add random deviate
    if add_deviate:
        deviated_position: list = add_deviate([x_position, y_position, orientation])
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
