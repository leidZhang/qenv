# 3rd party import
from abc import ABC, abstractmethod


class ServiceModule(ABC):
    """
    An abstract base class that defines the interface for service modules within the system.

    Service modules are components that provide specific functionalities and can be started, stopped, and checked for validity.

    Methods:
        terminate(): An abstract method that should be implemented to properly terminate the service.
        setup(*args): A method that can be overridden to set up the service with the given arguments.
        is_valid(): A method that checks if the service is in a valid state. Defaults to True.
        execute(*args): A method that can be overridden to execute the service's main functionality.
    """

    @abstractmethod
    def terminate(self) -> None:
        pass

    def setup(self, *args) -> None:
        pass

    def is_valid(self) -> bool:
        return True

    def execute(self, *args) -> None:
        pass


class Controller(ServiceModule):
    """
    An abstract base class for controllers, extending the ServiceModule class.

    Controllers are specialized service modules that manage the state of a system and provide normalization methods for input signals.

    Methods:
        set_state(state: dict): Sets the current state of the controller.
        normalize_throttle(x_axis_signal: int): An abstract method to normalize the throttle signal.
        normalize_steering(y_axis_signal: int): An abstract method to normalize the steering signal.
    """

    def set_state(self, state: dict) -> None:
        self.state:dict = state

    @abstractmethod
    def normalize_throttle(self, x_axis_signal: int) -> float:
        pass

    @abstractmethod
    def normalize_steering(self, y_axis_signal: int) -> float:
        pass
