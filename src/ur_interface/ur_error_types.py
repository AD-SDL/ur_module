"""Custom exception types for UR interface errors"""


class URConnectionError(Exception):
    """Custom exception for UR connection errors"""

    pass


class URMovementError(Exception):
    """Custom exception for UR movement errors"""

    pass


class GripperError(Exception):
    """Custom exception for gripper-related errors"""

    pass


class GripperConnectionError(Exception):
    """Custom exception for gripper connection errors"""

    pass


class GripperOperationError(Exception):
    """Custom exception for gripper operation errors"""

    pass
