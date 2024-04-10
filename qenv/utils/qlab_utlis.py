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
    """
    Spawns a series of basic shapes at specified waypoints in a Quanser Interactive Lab.

    This function iterates over a sequence of coordinates, spawning a basic shape at each location
    with a specified rotation and scale. It's designed to visualize waypoints in a 3D space.

    Parameters:
    - qlabs (QuanserInteractiveLabs): An instance of the QuanserInteractiveLabs class.
    - sequence (np.ndarray): A 2D numpy array where each column represents the x, y coordinates
      of a waypoint. The z-coordinate is fixed at 1.

    Returns:
    None: This function does not return any value.

    Note:
    The function assumes that the sequence array is properly formatted with each column
    representing a different waypoint.
    """
    rotation:list = [0, 0, 0]
    scale:list = [0.01, 0.01, 0.01]
    basic_shape: QLabsBasicShape = QLabsBasicShape(qlabs=qlabs, verbose=False)
    for i in range(len(sequence[0])):
        location = [sequence[0][i], sequence[1][i], 1]
        basic_shape.spawn_id(i, location=location, rotation=rotation, scale=scale, configuration=1, waitForConfirmation=False)

def get_random_deviate(boundry: float) -> float:
    """
    Generates a random float within a specified range.

    This function returns a random floating-point number N such that -boundry <= N <= boundry.
    The distribution is uniform, meaning each number within the specified range is equally likely to be returned.

    Parameters:
    - boundry (float): The boundary value for the range. The function will generate numbers from -boundry to boundry.

    Returns:
    float: A random floating-point number within the specified range.
    """
    return random.uniform(-boundry, boundry)

def cal_waypoint_angles(waypoint_sequence: np.ndarray, i) -> float:
    """
    Calculates the angle between two consecutive waypoints.

    Given a sequence of waypoints, this function computes the angle of the line connecting
    the ith waypoint to the (i+1)th waypoint relative to the positive x-axis. The angle is
    calculated in radians and takes into account the quadrant in which the line lies.

    Parameters:
    - waypoint_sequence (np.ndarray): An array of waypoints, where each waypoint is represented
      as a pair of x and y coordinates.
    - i (int): The index of the current waypoint in the sequence.

    Returns:
    float: The angle in radians between the current waypoint and the next one in the sequence.

    Note:
    The function assumes that the waypoint_sequence contains at least i+2 elements and that
    the index i is non-negative.
    """
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
    """
    Computes the angles between consecutive waypoints in a sequence.

    This function iterates through a sequence of waypoints and calculates the angle between
    each pair of consecutive waypoints using the 'cal_waypoint_angles' function. For the last
    waypoint, it duplicates the previous angle since there's no subsequent waypoint to form a pair.

    Parameters:
    - waypoint_sequence (np.ndarray): An array of waypoints, where each waypoint is represented
      as a pair of x and y coordinates.

    Returns:
    list: A list of angles in radians, one for each pair of consecutive waypoints in the sequence.
    """
    waypoint_angles: list = []
    for i in range(len(waypoint_sequence)):
        if i != len(waypoint_sequence) - 1:
            waypoint_angles.append(cal_waypoint_angles(waypoint_sequence, i))
        else:
            waypoint_angles.append(waypoint_angles[i-1])
    return waypoint_angles

def get_deviate_state(position: Union[list, np.ndarray]) -> list:
    """
    Applies a random deviation to a given position and orientation.

    This function takes a position with orientation and applies a random deviation to it.
    The deviation is calculated using a random value for both the position and the orientation.
    The new position is determined by adding the deviated distance to the original position
    in the direction of the original orientation.

    Parameters:
    - position (Union[list, np.ndarray]): The original position and orientation in the format [x, y, orientation].

    Returns:
    list: The new position and orientation after applying the deviation, in the format [x_position, y_position, orientation].

    Note:
    The function assumes that the 'position' parameter includes the orientation as the third element.
    """
    # get random deviate
    deviate: float = get_random_deviate(0.1)
    orientation_deviate: float = get_random_deviate(0.15)
    # deviated position
    x_position: float = position[0] + deviate * math.cos(position[2])
    y_position: float = position[1] + deviate * math.sin(position[2])
    orientation: float = position[2] + orientation_deviate

    return [x_position, y_position, orientation]

def spawn_on_node(roadmap: RoadMap, actor: QLabsActor, node_id: int = 2, add_deviate: bool = False) -> None:
    """
    Spawns an actor on a specified node within a roadmap, with an option to add deviation.

    This function places an actor at a specific node on a roadmap. If the 'add_deviate' flag is set to True,
    it applies a random deviation to the actor's position and orientation before spawning.

    Parameters:
    - roadmap (RoadMap): The roadmap containing the nodes.
    - actor (QLabsActor): The actor to be spawned.
    - node_id (int, optional): The ID of the node where the actor will be spawned. Defaults to 2.
    - add_deviate (bool, optional): A flag to determine whether to add deviation to the actor's position
        and orientation. Defaults to False.

    Raises:
    - Exception: If either 'roadmap' or 'actor' is None.

    Note:
    The function assumes that the roadmap and actor are valid and that the node_id exists within the roadmap.
    """
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
    actor.spawn_id(
        actorNumber=0,
        location=[x_position, y_position, 0],
        rotation=[0, 0, orientation],
        scale=[.1, .1, .1],
        configuration=0,
        waitForConfirmation=True
    )

def spawn_on_waypoints(waypoint_sequence: np.ndarray, waypoint_angles: list, actor: QLabsActor, waypoint_num: int = 0, add_deviate: bool = False) -> None:
    """
    Spawns an actor at a specified waypoint with an optional deviation applied.

    This function positions an actor at a given waypoint from a sequence. If 'add_deviate' is True,
    it applies a random deviation to the actor's position and orientation before spawning.

    Parameters:
    - waypoint_sequence (np.ndarray): An array of waypoints, each containing x and y coordinates.
    - waypoint_angles (list): A list of angles corresponding to the orientation at each waypoint.
    - actor (QLabsActor): The actor to be spawned.
    - waypoint_num (int, optional): The index of the waypoint where the actor will be spawned. Defaults to 0.
    - add_deviate (bool, optional): Whether to apply a random deviation to the position and orientation. Defaults to False.

    Raises:
    - Exception: If 'waypoint_sequence' or 'actor' is None.

    Note:
    The function assumes that 'waypoint_sequence' and 'waypoint_angles' are of the same length and that 'waypoint_num' is within
    the valid range of indices for 'waypoint_sequence'.
    """
    if waypoint_sequence is None or actor is None:
        raise Exception("parameters cannot be None")

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
    actor.spawn_id(
        actorNumber=0,
        location=[x_position, y_position, 0],
        rotation=[0, 0, orientation],
        scale=[.1, .1, .1],
        configuration=0,
        waitForConfirmation=True
    )
