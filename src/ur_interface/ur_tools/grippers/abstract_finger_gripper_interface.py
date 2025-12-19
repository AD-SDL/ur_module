"""Abstract finger gripper interface definitions."""

from abc import ABC, abstractmethod


class FingerGripper(ABC):
    """Abstract gripper interface."""

    @abstractmethod
    def open(self, pose: float, speed: float, force: float):
        """Open the gripper to a specified pose with given speed and force."""

    @abstractmethod
    def close(self, pose: float, speed: float, force: float):
        """Close the gripper to a specified pose with given speed and force."""
        raise NotImplementedError("This method should be overridden by subclasses.")
