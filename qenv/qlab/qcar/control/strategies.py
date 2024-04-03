from abc import ABC, abstractmethod


class VirtualControlStrategy(ABC):
    @abstractmethod
    def execute(self, state: dict) -> None:
        pass


class VirtualSafeStrategy(VirtualControlStrategy):
    def execute(self, state: dict) -> None:
        flag: bool = state['control_flags']['safe']

        if flag == True:
            state['throttle'] *= 0
        else:
            state['throttle'] *= 1


class VirtualReverseStrategy(VirtualControlStrategy):
    def execute(self, state: dict) -> None:
        if state['control_flags']['reverse']:
            state['throttle'] *= -1
            state['cruise_throttle'] *= -1
        else:
            state['throttle'] *= 1
            state['cruise_throttle'] *= 1


class VirtualCruiseStrategy(VirtualControlStrategy):
    def execute(self, state: dict) -> None:
        flag: bool = state['control_flags']['cruise']
        if flag == True:
            state['throttle'] = state['cruise_throttle']
