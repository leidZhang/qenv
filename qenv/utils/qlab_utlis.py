# 3rd party imports
import numpy as np
# quanser imports
from qvl.qlabs import QuanserInteractiveLabs
from qvl.basic_shape import QLabsBasicShape

def connect_to_qlab() -> QuanserInteractiveLabs:
    qlabs = QuanserInteractiveLabs()
    qlabs.open("localhost")
    return qlabs

def generate_waypoints(qlabs: QuanserInteractiveLabs, sequence: np.ndarray) -> None:
    rotation:list = [0, 0, 0]
    scale:list = [0.01, 0.01, 0.01]
    basic_shape: QLabsBasicShape = QLabsBasicShape(qlabs=qlabs, verbose=False)
    for i in range(len(sequence[0])):
        location = [sequence[0][i], sequence[1][i], 1]
        basic_shape.spawn_id(i, location=location, rotation=rotation, scale=scale, configuration=1, waitForConfirmation=False)