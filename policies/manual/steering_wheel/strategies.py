# other imports
import sys
import time
# 3rd party imports
from abc import ABC, abstractmethod
from logidrivepy import LogitechController
# custom imports
from .constants import BUTTON_UP_INDEX
from .constants import BUTTON_DOWN_INDEX
from .constants import BUTTON_A_INDEX
from .constants import BUTTON_XBOX_INDEX


class WheelControllerStrategy(ABC):
    """
    Abstract base class that defines the interface for wheel control strategies.

    Attributes:
        last_press (float): Timestamp of the last button press.

    Methods:
        execute(controller: LogitechController, state: dict, index: int): An abstract
        method that defines the execution behavior of the strategy.
    """

    def __init__(self) -> None:
        self.last_press: float = time.time()

    @abstractmethod
    def execute(self, controller: LogitechController, state: dict, index: int) -> None:
        pass


class WheelReverseFlagStrategy(WheelControllerStrategy):
    """
    Wheel control strategy for toggling the vehicle's reverse flag.

    Overrides:
        execute(controller: LogitechController, state: dict, index: int): Toggles the 'reverse'
        flag in the QCar's control state.
    """

    def execute(self, controller: LogitechController, state: dict, index: int) -> None:
        current_time: float = time.time()
        if controller.button_is_pressed(index, BUTTON_UP_INDEX) and current_time - self.last_press > 1:
            state['control_flags']['reverse'] = not state['control_flags']['reverse']
            self.last_press = current_time


class WheelCruiseFlagStrategy(WheelControllerStrategy):
    """
    Wheel control strategy for toggling the vehicle's cruise control flag.

    Overrides:
        execute(controller: LogitechController, state: dict, index: int): Toggles the 'cruise' flag
        and sets the 'cruise_throttle' in the QCar's control state.
    """

    def execute(self, controller: LogitechController, state: dict, index: int) -> None:
        current_time: float = time.time()
        if controller.button_is_pressed(index, BUTTON_DOWN_INDEX) and current_time - self.last_press > 1:
            state['control_flags']['cruise'] = not state['control_flags']['cruise']
            state['cruise_throttle']= state['throttle']
            self.last_press = current_time


class WheelLightFlagStrategy(WheelControllerStrategy):
    """
    Wheel control strategy for toggling the vehicle's light flag.

    Overrides:
        execute(controller: LogitechController, state: dict, index: int): Toggles the 'light' flag in the
        QCar's control state.
    """

    def execute(self, controller: LogitechController, state: dict, index: int) -> None:
        current_time: float = time.time()
        if controller.button_is_pressed(index, BUTTON_A_INDEX) and current_time - self.last_press > 1:
            state['control_flags']['light'] = not state['control_flags']['light']
            self.last_press = current_time


class WheelSafeFlagStrategy(WheelControllerStrategy):
    """
    Wheel control strategy for toggling the vehicle's safety flag.

    Overrides:
        execute(controller: LogitechController, state: dict, index: int): Toggles the 'safe' flag in the QCar's
        control state
    """

    def execute(self, controller: LogitechController, state: dict, index: int) -> None:
        current_time: float = time.time()
        if controller.button_is_pressed(index, BUTTON_XBOX_INDEX) and current_time - self.last_press > 1:
            state['control_flags']['safe'] = not state['control_flags']['safe']
            self.last_press = current_time
