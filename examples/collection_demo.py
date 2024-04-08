# python imports
import math
import time
import os
from multiprocessing import Process
# 3rd party imports
import cv2
import numpy as np
# quanser imports
from qvl.qcar import QLabsQCar
from qvl.qlabs import QuanserInteractiveLabs
# custom imports
from qenv.core import DataWriter
from qenv.core import NoImageException
from qenv.utils import connect_to_qlab
from qenv.qlab.env import ACCRoadMap # change import path here
from scripts.acc_2024.setup_competition import setup # change import path here
from scripts.acc_2024.traffic_lights_competition import traffic_lights_setup


class ImageDataWriter(DataWriter):
    """
    The ImageDataWriter class extends the DataWriter class to handle the writing of image data.

    This class provides functionality to save images to disk and record their file paths into a CSV file.

    Attributes:
        data_path (str): The directory path where image data will be stored.
        csv_filepath (str): The file path for the CSV file that records image paths.
        counter (int): A counter to keep track of the number of images saved.

    Methods:
        write_image(image: np.ndarray, counter: int) -> str:
            Saves the given image to the specified path and returns the full path of the image.
            Raises a NoImageException if the image is None.

        execute(image: np.ndarray, counter: int) -> None:
            Attempts to write the image to disk and record its path in the CSV file.
            If a NoImageException is caught, the method will pass without action.

    Raises:
        NoImageException: An error indicating that no image was provided for writing.
    """

    def write_image(self, image: np.ndarray, counter: int) -> str:
        if image is None:
            raise NoImageException()
        image_path:str = os.path.join(self.data_path, 'image_{}.jpg'.format(counter))
        print(f"Writing Image {counter}")
        cv2.imwrite(image_path, image)
        return image_path

    def execute(self, image: np.ndarray, counter: int) -> None:
        try:
            image_path:str = self.write_image(image=image, counter=counter)
            self.record_data([image_path, 0])
        except NoImageException:
            pass

def spawn_on_seq(car: QLabsQCar, waypoint_sequence: np.ndarray, degree=0, waypoint_id: int = 0) -> None:
    x_position = waypoint_sequence[waypoint_id][0]
    y_position = waypoint_sequence[waypoint_id][1]
    car.spawn_id(actorNumber=0, location=[x_position, y_position, 0], rotation=[0, 0, degree], scale=[.1, .1, .1], configuration=0, waitForConfirmation=True)

def collect_trafficlight_data() -> None:
    # create map
    setup() # modified setup function in Setup_Competition, commmented out car2.spawn_id and car2.possess()
    writer: ImageDataWriter = ImageDataWriter()
    writer.setup() # can change folder and csv file name with this function

    qlabs: QuanserInteractiveLabs = connect_to_qlab()
    car: QLabsQCar = QLabsQCar(qlabs=qlabs)
    road_map: ACCRoadMap = ACCRoadMap()
    node_sequence: list = [10, 2, 4, 14, 20, 22, 10]
    waypoint_sequence: np.ndarray = road_map.generate_path(node_sequence)
    print(waypoint_sequence)
    degree: float = math.pi / 4
    spawn_on_seq(car=car, waypoint_sequence=waypoint_sequence, waypoint_id=410, degree=degree)
    Process(target=traffic_lights_setup).start()
    time.sleep(5)
    print("Start Recording...")
    image_counter: int = 0
    while degree <= 3 * math.pi / 4:
        spawn_on_seq(car=car, waypoint_sequence=waypoint_sequence, waypoint_id=410, degree=degree)
        time.sleep(2) # may need to sleep more time
        image: np.ndarray = car.get_image(3)[1]
        writer.execute(image=image, counter=image_counter)
        print(f'Image {image_counter} recorded')
        time.sleep(2) # may need to sleep more time
        # cv2.destroyAllWindows()
        image_counter += 1
        car.destroy()
        degree += math.pi / 180
    print("Collection complete!")
