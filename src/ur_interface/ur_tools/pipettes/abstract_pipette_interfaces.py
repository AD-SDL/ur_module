"""Abstract gripper interface definitions."""

from abc import ABC, abstractmethod


class Pipette(ABC):
    """Abstract gripper interface."""

    @abstractmethod
    def activate(self):
        """activates the gripper"""
    @abstractmethod
    def aspirate(self, volume: float):
        """Aspirate a specified volume at a given speed."""
    @abstractmethod
    def dispense(self, volume: float):
        """Dispense a specified volume at a given speed."""
    