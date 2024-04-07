# 3rd party imports
import numpy as np
# quanser imports
from qvl.qlabs import QuanserInteractiveLabs
from hal.utilities.path_planning import RoadMapNode
# custom imports
from qenv.qlab.env import ACCRoadMap
from qenv.utils.qlab_utlis import connect_to_qlab
from scripts.acc_2024.setup_competition import setup

def spawn_demo(id=24) -> None:
    roadmap: ACCRoadMap = ACCRoadMap()
    node_id: int = id # change node_id here
    node: RoadMapNode = roadmap.nodes[node_id]
    setup(initialPosition=[node.pose[0], node.pose[1], 0], initialOrientation=[0, 0, node.pose[2]])

    # qlabs: QuanserInteractiveLabs = QuanserInteractiveLabs()
    # qlabs.open("localhost")
    # node_sequence: list = [10, 17, 10]
    # waypoint_sequence: np.ndarray = roadmap.generate_path(node_sequence)
