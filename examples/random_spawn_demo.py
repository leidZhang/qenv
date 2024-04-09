# python imports
import time
# 3rd party imports
import numpy as np
# quanser imports
from qvl.qcar import QLabsQCar
from qvl.qlabs import QuanserInteractiveLabs
# custom imports
from qenv.qlab.env import ACCRoadMap
from qenv.utils.qlab_utlis import get_waypoint_angles
from qenv.utils.qlab_utlis import connect_to_qlab
from qenv.utils.qlab_utlis import spawn_on_waypoints
from scripts.acc_2024.setup_competition import setup

def random_spawn_demo() -> None:
    setup() # generate map

    # init instances
    roadmap: ACCRoadMap = ACCRoadMap()
    qlabs: QuanserInteractiveLabs = connect_to_qlab()
    qlabs.open("localhost")
    car: QLabsQCar = QLabsQCar(qlabs)
    waypoints: np.ndarray = roadmap.generate_path([10, 2, 4, 14, 20, 22, 10])
    angles: list = get_waypoint_angles(waypoints)

    counter = 10
    while counter >= 0:
        spawn_on_waypoints(waypoints, angles, car, add_deviate=False)
        car.possess()
        time.sleep(2)
        car.destroy()

    print("Demo complete")
    qlabs.close()
