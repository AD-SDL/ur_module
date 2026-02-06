"""Abstract gripper interface definitions."""

from abc import ABC, abstractmethod


class Gripper(ABC):
    """Abstract gripper interface."""

    @abstractmethod
    def activate(self):
        """activates the gripper"""


class FingerGripper(Gripper):
    """Abstract gripper interface."""

    @abstractmethod
    def open(self, pose: float, speed: float, force: float):
        """Open the gripper to a specified pose with given speed and force."""

    @abstractmethod
    def close(self, pose: float, speed: float, force: float):
        """Close the gripper to a specified pose with given speed and force."""
