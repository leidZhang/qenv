# othre imports
import os
# 3rd party imports
import keyboard
from typing import List
# custom imports
from qenv.core import Controller
from qenv.core import QCAR_CONTROL_PROTOCAL
from .strategies import *
from .constants import *


class KeyboardController(Controller):
    """
    A controller class designed to interface with a QCar model using keyboard inputs.

    Attributes:
        x_axis_signal (float): The current signal value for the x-axis, representing throttle.
        y_axis_signal (float): The current signal value for the y-axis, representing steering.
        mode (str): The mode of operation, defaulting to 'QCar'.
        state (dict): The current control state of the QCar.
        press_strategies (List[KeyboardControllerStrategy]): A list of strategies for handling keyboard presses.

    Methods:
        terminate(): Exits the program.
        to_zero(val: int): Decreases the value towards zero by NORMAL_DECREASE.
        normalize_steering(y_axis_signal: int): Normalizes the steering signal.
        normalize_throttle(x_axis_signal: int): Normalizes the throttle signal.
        setup(): Initializes the controller state and press strategies.
        execute(): Processes keyboard inputs and updates the QCar state accordingly.
    """

    def __init__(self, mode: str = 'QCar') -> None:
        self.x_axis_signal: float = 0
        self.y_axis_signal: float = 0
        self.mode: str = mode
        self.state: dict = None
        self.press_strategies: List[KeyboardControllerStrategy] = None

    def terminate(self) -> None:
        os._exit(0)

    def to_zero(self, val: int) -> int:
        if val > 0:
            return val - NORMAL_DECREASE
        elif val < 0:
            return val + NORMAL_DECREASE
        else:
            return 0

    def normalize_steering(self, y_axis_signal: int) -> float:
        return y_axis_signal / MAX_Y_AXIS_VALUE

    def normalize_throttle(self, x_axis_signal: int) -> float:
        return x_axis_signal / MAX_X_AXIS_VALUE

    def setup(self) -> None:
        if self.mode == 'QCar':
            self.state = QCAR_CONTROL_PROTOCAL
            self.press_strategies = [
                KeyboardSafeFlagStrategy(self.state, '/'),
                KeyboardReverseFlagStrategy(self.state, 'e'),
                KeyboardLightFlagStrategy(self.state, 'l'),
                KeyboardCruiseFlagStrategy(self.state, 'q'),
            ]
            for strategy in self.press_strategies:
                keyboard.on_press_key(strategy.key, strategy.execute)
        else:
            raise ValueError('Control Protocol cannot be found')

    def execute(self) -> None:
        if keyboard.is_pressed('w'):
            self.x_axis_signal += X_AXIS_DECREASE
            if self.x_axis_signal > MAX_X_AXIS_VALUE:
                self.x_axis_signal = MAX_X_AXIS_VALUE
        elif keyboard.is_pressed('s'):
            self.x_axis_signal = 0
        else:
            self.x_axis_signal = self.to_zero(self.x_axis_signal)

        if keyboard.is_pressed('a'):
            self.y_axis_signal += Y_AXIS_DECREASE
            if self.y_axis_signal > MAX_Y_AXIS_VALUE:
                self.y_axis_signal = MAX_Y_AXIS_VALUE
        elif keyboard.is_pressed('d'):
            self.y_axis_signal -= Y_AXIS_DECREASE
            if self.y_axis_signal < -MAX_Y_AXIS_VALUE:
                self.y_axis_signal = -MAX_Y_AXIS_VALUE
        else:
            self.y_axis_signal = self.to_zero(self.y_axis_signal)

        if keyboard.is_pressed('`'):
            self.terminate()

        if self.mode == 'QCar':
            self.state['throttle'] = self.normalize_throttle(self.x_axis_signal)
            self.state['steering'] = self.normalize_steering(self.y_axis_signal)