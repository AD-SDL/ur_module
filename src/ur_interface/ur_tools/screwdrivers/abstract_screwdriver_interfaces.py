"""Abstract gripper interface definitions."""

from abc import ABC, abstractmethod
from enum import Enum

class Direction(str, Enum):
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"
class Screwdriver(ABC):
    """Abstract gripper interface."""

    @abstractmethod
    def activate(self):
        """activates the gripper"""
    @abstractmethod
    def turn_screwdriver(self, direction: Direction, duration: int=120):
        """Aspirate a specified volume at a given speed."""

    