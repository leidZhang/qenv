# other imports
from typing import List
# 3rd party imports
import numpy as np
# quanser imports
from pal.products.qcar import QCar
# custom imports
from .strategies import *
from .constants import *
from qenv.core import ServiceModule


class VirtualControl(ServiceModule):
    """
    A virtual controller for managing the state and behavior of a QCar simulation.

    This class integrates various control strategies to operate a virtual QCar, handling
    its state, LED indicators, and executing defined control strategies.

    Attributes:
        state (dict): The current state of the QCar, including control flags and signals.
        my_car (QCar): An instance of the QCar class for simulation interaction.
        leds (np.ndarray): An array representing the state of the QCar's LEDs.
        strategies (List[VirtualControlStrategy]): A list of control strategies to be applied to the QCar.

    Methods:
        unlock(state: dict): Unlocks the QCar for non-manual control policies.
        handle_leds(): Updates the LEDs based on the current steering direction and other control flags.
        terminate(): Terminates the QCar simulation.
        execute(state: dict): Executes the control strategies and updates the QCar's state in the simulation.
    """

    def __init__(self) -> None:
        self.state: dict = None
        self.my_car: QCar = QCar(readMode=0)
        self.leds: np.ndarray = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.strategies: List[VirtualControlStrategy] = [
            VirtualCruiseStrategy(),
            VirtualReverseStrategy(),
            VirtualSafeStrategy(),
        ]

    def unlock(self, state: dict) -> None:
        # for non-manul policies
        state['control_flags']['safe'] = state
        self.state = state

    def handle_leds(self) -> None:
        # lights related to steering
        if self.state['steering'] > 0.3:
            self.leds[0] = 1
            self.leds[2] = 1
        elif self.state['steering'] < -0.3:
            self.leds[1] = 1
            self.leds[3] = 1
        else:
            self.leds = np.array([0, 0, 0, 0, 0, 0, self.leds[6], self.leds[7]])
        # other lights
        if self.state['control_flags']['light']:
            self.leds[6] = 1
            self.leds[7] = 1
        else:
            self.leds[6] = 0
            self.leds[7] = 0

    def terminate(self) -> None:
        self.my_car.terminate()

    def execute(self, state: dict) -> None:
        self.state = state
        # execute strategies
        for strategy in self.strategies:
            strategy.execute(self.state)

        # handle control
        self.state['throttle'] = QCAR_MAX_THROTTLE * self.state['throttle']
        self.state['steering'] = QCAR_MAX_STEERING * self.state['steering']
        # handle leds
        self.handle_leds()
        # write state to virtual qcar
        self.my_car.read_write_std(
            self.state['throttle'],
            self.state['steering'],
            self.leds
        )
