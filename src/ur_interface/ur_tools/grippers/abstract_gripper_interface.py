"""Abstract gripper interface definitions."""

class Gripper:
    """Abstract gripper interface."""

    def open(self, pose: float, speed: float, force: float):
        """Open the gripper to a specified pose with given speed and force."""
        raise NotImplementedError("This method should be overridden by subclasses.")

    def close(self, pose: float, speed: float, force: float):
        """Close the gripper to a specified pose with given speed and force."""
        raise NotImplementedError("This method should be overridden by subclasses.")