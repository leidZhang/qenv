# other imports
import os
import math
import time
# 3rd party imports
import cv2
import numpy as np
# quanser imports
from pal.products.qcar import QCarGPS
# from pal.utilities.lidar import Lidar
from pal.utilities.vision import Camera2D
from pal.utilities.vision import Camera3D
# custom imports
from qenv.core import ServiceModule
from .constants import CSI_CAMERA_SETTING, RGBD_CAMERA_SETTING


class VirtualCSICamera(ServiceModule): # wrapper class, implement more functions if needed
    """
    A service module that provides an interface to a CSI camera for image capture in a virtual
    environment.

    The class encapsulates the functionality of a 2D camera, allowing for image capture, display,
    and retrieval with optional debugging features.

    Attributes:
        id (int): The identifier for the camera, defaulting to 3.
        debug (bool): A flag indicating whether debugging features like image display are enabled.
        camera (Camera2D): An instance of the Camera2D class configured for CSI camera settings.

    Methods:
        terminate(): Shuts down the camera and releases any associated resources.
        show_image(): Displays the current image from the camera if debugging is enabled.
        read_image(): Reads the latest image from the camera and returns it as a numpy ndarray.
    """

    def __init__(self, id: int = 3, debug: bool = False) -> None:
        self.id: int = id
        self.debug: bool = debug
        self.camera: Camera2D = Camera2D(
            cameraId=str(id) + "@tcpip://localhost:"  + str(18961+id),
            frameWidth=CSI_CAMERA_SETTING['frame_width'],
            frameHeight=CSI_CAMERA_SETTING['frame_height'],
            frameRate=CSI_CAMERA_SETTING['frame_rate'],
            focalLength=CSI_CAMERA_SETTING['focal_length'],
            principlePoint=CSI_CAMERA_SETTING['principle_point'],
            position=CSI_CAMERA_SETTING['position'],
            orientation=CSI_CAMERA_SETTING['orientation']
        )

    def terminate(self) -> None:
        self.camera.terminate()

    def show_image(self) -> None:
        if self.debug:
            cv2.imshow(f'CSI Camera {self.id}', self.camera.imageData)

    def read_image(self) -> np.ndarray: # image without any modification
        if self.camera.read():
            self.show_image()
            return self.camera.imageData
        return None


class VirtualRGBDCamera(ServiceModule):
    """
    A service module for interfacing with a virtual RGB-D camera within a simulation environment.

    This class provides methods to capture, display, and retrieve RGB and depth images from a
    simulated 3D camera, with optional debugging features for visualizing the captured images.

    Attributes:
        debug (bool): Indicates whether debugging features like image display are enabled.
        camera (Camera3D): An instance of the Camera3D class configured with RGB-D camera settings.

    Methods:
        terminate(): Shuts down the camera and releases any associated resources.
        show_image_rgb(): Displays the current RGB image from the camera if debugging is enabled.
        read_rgb_image(): Captures the latest RGB image from the camera and returns it as a numpy
            ndarray.
        show_image_depth(mode: str): Displays the current depth image in the specified mode if
            debugging is enabled.
        read_depth_image(mode: str): Captures the latest depth image in the specified mode from the
            camera and returns it as a numpy ndarray.
        read_image(mode: str): Captures either the latest RGB or depth image based on the mode
            specified and returns it as a numpy ndarray.
    """

    def __init__(self, debug: bool = False) -> None:
        self.debug: bool = debug
        self.camera: Camera3D = Camera3D(
            mode=RGBD_CAMERA_SETTING['mode'],
            frameWidthRGB=RGBD_CAMERA_SETTING['frame_width_rgb'],
            frameHeightRGB=RGBD_CAMERA_SETTING['frame_height_rgb'],
            frameRateRGB=RGBD_CAMERA_SETTING['frame_rate_rgb'],
            frameWidthDepth=RGBD_CAMERA_SETTING['frame_width_depth'],
            frameHeightDepth=RGBD_CAMERA_SETTING['frame_height_depth'],
			frameRateDepth=RGBD_CAMERA_SETTING['frame_rate_depth'],
            deviceId=RGBD_CAMERA_SETTING['device_id']
        )

    def terminate(self) -> None:
        self.camera.terminate()

    def show_image_rgb(self) -> None:
        if self.debug:
            cv2.imshow('RGBD Image', self.camera.imageBufferRGB)

    def read_rgb_image(self) -> np.ndarray:
        if self.camera.read_RGB() != -1:
            self.show_image_rgb()
            return self.camera.imageBufferRGB
        return None

    def show_image_depth(self, mode: str) -> None:
        if self.debug:
            if mode == 'PX':
                cv2.imshow('RGBD PX', self.camera.imageBufferDepthPX)
            else:
                cv2.imshow('RGBD M', self.camera.imageBufferDepthM)

    def read_depth_image(self, mode:str) -> np.ndarray:
        if self.camera.read_depth(mode) != -1:
            if mode == 'PX':
                self.show_image_depth(mode)
                return self.camera.imageBufferDepthPX
            if mode == 'M':
                self.show_image_depth(mode)
                return  self.camera.imageBufferDepthM
            return None

    def read_image(self, mode: str='') -> np.ndarray:
        if mode == '':
            return self.read_rgb_image()
        return self.read_depth_image(mode)


class VirtualLidar:
    """
    Got some issue here
    """

    def __init__(self) -> None:
        pass


class VirtualGPS(ServiceModule):
    """
    A virtual GPS module for the QCar simulation environment, providing position and velocity data.

    This class simulates a GPS device that tracks the position and calculates the speed vector of a
    QCar based on its movement over time.

    Attributes:
        gps (QCarGPS): An instance of the QCarGPS class to simulate GPS functionality.
        time_stamp (float): The last recorded time stamp for position data.
        delta_t (float): The time difference between the last and current state readings.
        current_state (np.ndarray): The current position and orientation of the QCar.
        last_state (np.ndarray): The previous position and orientation of the QCar.
        speed_vector (list): The calculated speed vector representing linear and angular speeds.

    Methods:
        terminate(): Terminates the GPS simulation.
        get_gps_state(): Retrieves the current position and orientation of the QCar from the GPS.
        calculate_speed_vector(current_state: np.ndarray, delta_t: float): Calculates the speed vector
            based on the current and last states.
        setup(): Initializes the GPS module, creating or overwriting the GPS log, and sets the initial state.
        read_gps_state(): Reads the current GPS state, calculates the speed if necessary, and updates
            the state and time stamp.
    """

    def __init__(self, debug: bool = False) -> None:
        self.debug: bool = debug
        self.gps: QCarGPS = QCarGPS()
        self.time_stamp: float
        self.delta_t: float
        self.current_state: np.ndarray
        self.last_state: np.ndarray
        self.speed_vector: list

    def terminate(self) -> None:
        self.gps.terminate()

    def get_gps_state(self) -> tuple[float, float, float]:
        position_x: float = self.gps.position[0]
        position_y: float = self.gps.position[1]
        orientation: float = self.gps.orientation[2]

        return position_x, position_y, orientation

    def calculate_speed_vector(self, current_state: np.ndarray, delta_t: float) -> tuple[float, float]:
        # calculate differents
        delta_x_sq: float = math.pow((current_state[0] - self.last_state[0]), 2)
        delta_y_sq: float  = math.pow((current_state[1] - self.last_state[1]), 2)
        # calculate speed
        linear_speed: float  = math.pow((delta_x_sq + delta_y_sq), 0.5) / delta_t
        angular_speed: float = (current_state[2] - self.last_state[2]) / delta_t

        return linear_speed, angular_speed

    def setup(self) -> None:
        # create or overwrite the log
        open("output/gps_log.txt", "w")
        # init states
        self.time_stamp = time.time()
        self.gps.readGPS() # read gps info
        self.last_state = self.get_gps_state()

    def read_gps_state(self) -> None:
        # read current position
        self.gps.readGPS()
        current_time = time.time()
        self.current_state = self.get_gps_state()
        # calculate absolute speed
        if self.current_state != self.last_state or current_time - self.time_stamp >= 0.25:
            self.delta_t = current_time - self.time_stamp
            self.speed_vector = self.calculate_speed_vector(self.current_state, self.delta_t)
            if self.debug:
                os.system("cls")
                print(f"last position: {self.last_state}")
                print(f"current position: {self.current_state}")
                print(f"dt: {self.delta_t}, state: {self.speed_vector}")
            # update time stamp
            self.time_stamp = current_time
            # update position
            self.last_state = self.current_state