# other imports
import os
# 3rd party imports
import cv2
from typing import Union
from multiprocessing import Process, Queue
# custom imports
from examples.policies.manual.keyboard import KeyboardController
from qenv.utils.qlab_utlis import connect_to_qlab
from qenv.qlab.qcar.sensor import VirtualCSICamera
from qenv.qlab.qcar import VirtualBaseQCar


class VirtualManualQCar(VirtualBaseQCar):
    """
    Represents a virtual manual control car, inheriting from VirtualBaseQCar.

    Attributes:
        csi_cameras (dict): A dictionary storing the VirtualCSICamera objects associated with the car.

    Methods:
        setup(): Sets up the virtual car by initializing its CSI cameras. Specifically, it creates a
            front-facing camera with debugging enabled.
        execute_modules(input: Union[Queue, dict]): Executes the modules associated with the virtual car.
            This method overrides the execute_modules method from the base class to perform additional
            actions, such as reading images from the CSI cameras.
    """

    def __init__(self) -> None:
        super().__init__()

    def setup(self) -> None:
        # Can add more modules here
        self.csi_cameras['front'] = VirtualCSICamera(id=3, debug=True)

    def execute_modules(self, input: Union[Queue, dict]) -> None:
        super().execute_modules(input)
        # read csi images
        for camera in self.csi_cameras.values():
            camera.read_image()

def run_qcar(queue: Queue) -> None:
    try:
        connect_to_qlab()
        car = VirtualManualQCar()
        car.setup()

        while True:
            car.execute_modules(queue)
            if cv2.waitKey(1) & 0xFF == ord('m'):
                break
    except Exception as e:
        print(e)
        car.terminate()
        os._exit(0)

def interprocess_demo() -> None:
    try:
        # init instances
        queue: Queue = Queue(maxsize=5)
        process: Process = Process(target=run_qcar, args=(queue, ))
        policy: KeyboardController = KeyboardController()
        # setup process
        policy.setup()
        process.start()
        # execute policy
        while True:
            policy.execute(queue)
    except KeyboardInterrupt:
        print("End demo")
        process.terminate()
        os._exit(0)
    except Exception as e:
        print(e)
