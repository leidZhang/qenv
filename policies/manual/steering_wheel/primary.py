# other imports
import os
import sys
# 3rd party imports
from typing import List
from logidrivepy import LogitechController
from multiprocessing import Queue
# custom imports
from qenv.core import Controller
from qenv.core import QCAR_CONTROL_PROTOCAL
from qenv.core import handle_interprocess
from .constants import *
from .strategies import *


class WheelController(Controller):
    """
    A controller class designed to interface with a QCar model using a Logitech steering wheel controller.

    Attributes:
        index (int): The index of the controller, defaulting to 0.
        state (dict): The current control state of the QCar, following the QCAR_CONTROL_PROTOCOL.
        controller (LogitechController): An instance of the LogitechController to interact with the physical device.
        control_strategies (List[WheelControllerStrategy]): A list of strategies for controlling various flags like safety, reverse, light, and cruise.

    Methods:
        terminate(): Shuts down the steering controller.
        normalize_steering(y_axis_signal: int): Normalizes the steering signal from the controller.
        normalize_throttle(x_axis_signal: int): Normalizes the throttle signal from the controller.
        setup(): Initializes the steering controller and checks for its connection.
        execute(queue: Queue): Updates the state of the QCar model based on input from the wheel controller and applies control strategies.

    Raises:
        ValueError: If the mode is not set to 'QCar'.
        ConnectionRefusedError: If the LogitechController is not connected.
    """

    def __init__(self, index:str = "0", mode: str = 'QCar') -> None:
        # check mode
        if mode != 'QCar':
            raise ValueError("Steering wheel controller can only control QCar")
        self.index: int = int(index)
        self.state: dict = QCAR_CONTROL_PROTOCAL
        self.controller: LogitechController = LogitechController()
        # add more strategies if needed
        self.control_strategies: List[WheelControllerStrategy] = [
            WheelSafeFlagStrategy(),
            WheelReverseFlagStrategy(),
            WheelLightFlagStrategy(),
            WheelCruiseFlagStrategy(),
        ]

    def terminate(self) -> None:
        self.controller.steering_shutdown()

    def normalize_steering(self, y_axis_signal: int) -> float:
        return y_axis_signal / WHEEL_CONTROLLER_STEERING_MIN

    def normalize_throttle(self, x_axis_signal: int) -> float:
        return (WHEEL_CONTROLLER_ACCELERATOR_MID - x_axis_signal) / WHEEL_CONTROLLER_ACCELERATOR_MAX

    def setup(self) -> None:
        self.controller.steering_initialize(True)
        if not self.controller.is_connected(self.index):
            raise ConnectionRefusedError("Device not connected!")

    def execute(self, queue: Queue) -> None:
        if self.controller.logi_update(): # update every frame
            state_engines = self.controller.get_state_engines(self.index) # get input from the wheel controller
            self.state['throttle'] = self.normalize_throttle(state_engines.contents.lY)
            self.state['steering'] = self.normalize_steering(state_engines.contents.lX)
            # light toggle, reverse, cruise, etc
            for strategy in self.control_strategies:
                strategy.execute(self.controller, self.state, self.index)
            # push to the queue
            handle_interprocess(queue, self.state)