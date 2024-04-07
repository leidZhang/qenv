# third party imports
from typing import List, Union
from multiprocessing.queues import Queue
# custom imports
from qenv.qlab.qcar.control import VirtualControl
from qenv.qlab.qcar.sensor import VirtualCSICamera, VirtualRGBDCamera


class VirtualBaseQCar:
    """
    The VirtualBaseQCar class simulates a virtual car's control system, including
    its motion apparatus, cameras, and other sensors.

    Attributes:
        running_gear (VirtualControl): Handles the virtual car's motion control.
        csi_cameras (dict[VirtualCSICamera]): A collection of CSI cameras for capturing
            environmental imagery.
        rgbd_camera (VirtualRGBDCamera): An RGB-D camera for capturing color images and
            depth information.

    Methods:
        __init__(): Initializes the virtual car's control systems and sensors.
        terminate(): Terminates all sensors and motion apparatus, ensuring proper resource
            release.
        execute_modules(input: Union[Queue, dict]): Controls the virtual car's behavior
            based on the provided commands.

    This class allows for the control of the virtual car's movement through policy inputs or
    programmatic commands and is capable of handling commands from a multiprocessing queue.
    """

    def __init__(self) -> None:
        self.running_gear: VirtualControl = VirtualControl()
        self.csi_cameras: dict[str, VirtualCSICamera] = {}
        self.rgbd_cameara: VirtualRGBDCamera = None
        self.command: dict = None

    def terminate(self) -> None:
        # shut down csi cameras
        if self.csi_cameras:
            for camera in self.csi_cameras.values():
                camera.terminate()
        # shut down rgbd cameras
        if self.rgbd_cameara:
            self.rgbd_cameara.terminate()
        # shut down running gear
        self.running_gear.terminate()

    def execute_modules(self, input: Union[Queue, dict]) -> None:
        # check None
        if input is None:
            raise ValueError("Incorrect input to qcar")
        # check multiprocess
        if isinstance(input, Queue):
            if not input.empty():
                self.command = input.get()
        else:
            self.command = input
        # send the command to the running gear
        self.running_gear.execute(self.command)
