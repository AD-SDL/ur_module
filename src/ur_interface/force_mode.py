"""
Clean UR Robot Calibration System using ur_rtde
Uses ur_rtde's built-in moveUntilContact() for proper contact detection
"""

# flake8: noqa
import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

import numpy as np

try:
    import rtde_control
    import rtde_receive

    RTDE_AVAILABLE = True
    print("✅ ur_rtde successfully imported!")
except ImportError as e:
    RTDE_AVAILABLE = False
    print(f"❌ ur_rtde not available: {e}")
    print("Please install with: pip install ur_rtde")


class CalibrationDirection(Enum):
    """Defines the approach directions for calibration"""

    POSITIVE_X = "pos_x"
    NEGATIVE_X = "neg_x"
    POSITIVE_Y = "pos_y"
    NEGATIVE_Y = "neg_y"
    POSITIVE_Z = "pos_z"
    NEGATIVE_Z = "neg_z"


@dataclass
class CalibrationConfig:
    """Configuration parameters for calibration process"""

    approach_speed: float = 0.01  # m/s - speed when approaching target
    retreat_distance: float = 0.005  # meters - distance to retreat after contact
    approach_distance: float = 0.03  # meters - distance to approach from
    acceleration: float = 0.1  # m/s² - acceleration for movements
    contact_timeout: float = 15.0  # seconds - timeout for contact detection


@dataclass
class CalibrationResult:
    """Results from a calibration operation"""

    success: bool
    calibrated_position: Optional[List[float]]
    contact_positions: Dict[CalibrationDirection, List[float]]
    contact_forces: Dict[CalibrationDirection, float]
    errors: List[str]


class URCalibrationRTDE:
    """
    Clean calibration system using ur_rtde's built-in contact detection
    """

    def __init__(self, robot_ip: str, config: CalibrationConfig = None):
        """
        Initialize RTDE calibration system

        Args:
            robot_ip: IP address of the UR robot
            config: Calibration configuration
        """
        if not RTDE_AVAILABLE:
            raise ImportError("ur_rtde library not available. Please install it first.")

        self.robot_ip = robot_ip
        self.config = config or CalibrationConfig()
        self.logger = logging.getLogger("ur_calibration_rtde")

        # Initialize RTDE connections
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

        self.logger.info(f"✅ RTDE connected to robot at {robot_ip}")

    def calibrate_target_location(
        self,
        approximate_target: List[float],
        approach_directions: List[CalibrationDirection] = None,
        tcp_offset: List[float] = None,
    ) -> CalibrationResult:
        """
        Calibrate target location using RTDE moveUntilContact

        Args:
            approximate_target: [x, y, z, rx, ry, rz] approximate target pose
            approach_directions: List of directions to approach from
            tcp_offset: Optional TCP offset for calibration tool

        Returns:
            CalibrationResult with calibrated position and details
        """
        if approach_directions is None:
            approach_directions = [CalibrationDirection.POSITIVE_X, CalibrationDirection.NEGATIVE_X]

        self.logger.info(f"🎯 Starting calibration for target: {approximate_target}")

        result = CalibrationResult(
            success=False, calibrated_position=None, contact_positions={}, contact_forces={}, errors=[]
        )

        try:
            # Set TCP if provided
            if tcp_offset:
                self.rtde_c.setTcp(tcp_offset)
                self.logger.info(f"🔧 TCP offset set: {tcp_offset}")
                time.sleep(0.5)

            # Store initial position for safety return
            initial_pose = self.rtde_r.getActualTCPPose()

            # Perform calibration from each direction
            for i, direction in enumerate(approach_directions):
                self.logger.info(f"📍 Calibrating from direction {i + 1}/{len(approach_directions)}: {direction.value}")

                contact_result = self._calibrate_from_direction(approximate_target, direction)

                if contact_result["success"]:
                    result.contact_positions[direction] = contact_result["contact_position"]
                    result.contact_forces[direction] = contact_result["contact_force"]
                    self.logger.info(f"✅ Contact successful: force={contact_result['contact_force']:.2f}N")
                else:
                    result.errors.append(f"Failed from {direction.value}: {contact_result['error']}")
                    self.logger.warning(f"❌ Contact failed: {contact_result['error']}")

            # Calculate final calibrated position
            if len(result.contact_positions) >= 2:
                result.calibrated_position = self._calculate_calibrated_position(
                    result.contact_positions, approximate_target
                )
                result.success = True
                self.logger.info(f"🎉 Calibration successful! Final position: {result.calibrated_position}")
            else:
                result.errors.append("Insufficient successful contact points for calibration")
                self.logger.error("💥 Calibration failed: insufficient contact points")

            # Return to safe initial position
            self.rtde_c.moveL(initial_pose, self.config.approach_speed, self.config.acceleration)
            self.logger.info("🏠 Returned to initial position")

        except Exception as e:
            result.errors.append(f"Calibration exception: {str(e)}")
            self.logger.error(f"💥 Calibration failed with exception: {e}")

        return result

    def _calibrate_from_direction(self, target: List[float], direction: CalibrationDirection) -> Dict:
        """
        Perform calibration approach from a specific direction using RTDE
        """
        try:
            # Calculate approach position
            approach_pose = self._calculate_approach_pose(target, direction)

            # Move to approach position
            self.logger.info(f"🚀 Moving to approach position: {approach_pose[:3]}")
            self.rtde_c.moveL(approach_pose, self.config.approach_speed, self.config.acceleration)

            # Wait for movement to complete
            while not self.rtde_r.isRobotStopped():
                time.sleep(0.01)

            time.sleep(0.5)  # Allow settling

            # Get direction for contact detection
            contact_direction = self._get_contact_direction_vector(direction)

            # Use RTDE's built-in moveUntilContact - this is the magic! 🪄
            self.logger.info(f"👋 Starting contact detection in direction: {contact_direction[:3]}")
            contact_detected = self.rtde_c.moveUntilContact(
                xd=contact_direction, direction=contact_direction, acceleration=self.config.acceleration
            )

            if contact_detected:
                # Get contact position and force
                contact_position = self.rtde_r.getActualTCPPose()
                tcp_force = self.rtde_r.getActualTCPForce()
                contact_force = np.linalg.norm(tcp_force[:3])  # Force magnitude

                self.logger.info(f"🎯 Contact detected at: {contact_position[:3]}")

                # Retreat slightly for safety
                retreat_pose = self._calculate_retreat_pose(contact_position, direction)
                self.rtde_c.moveL(retreat_pose, self.config.approach_speed / 2, self.config.acceleration)

                return {
                    "success": True,
                    "contact_position": contact_position,
                    "contact_force": contact_force,
                    "error": None,
                }
            else:
                return {
                    "success": False,
                    "contact_position": None,
                    "contact_force": 0.0,
                    "error": f"No contact detected within timeout from {direction.value}",
                }

        except Exception as e:
            return {"success": False, "contact_position": None, "contact_force": 0.0, "error": str(e)}

    def _get_contact_direction_vector(self, direction: CalibrationDirection) -> List[float]:
        """
        Get direction vector for moveUntilContact
        Format: [x, y, z, rx, ry, rz] where distance components are in meters
        """
        # Use small movement distance for contact detection
        move_distance = 0.05  # 5cm max movement for contact detection

        direction_vectors = {
            CalibrationDirection.POSITIVE_X: [move_distance, 0, 0, 0, 0, 0],
            CalibrationDirection.NEGATIVE_X: [-move_distance, 0, 0, 0, 0, 0],
            CalibrationDirection.POSITIVE_Y: [0, move_distance, 0, 0, 0, 0],
            CalibrationDirection.NEGATIVE_Y: [0, -move_distance, 0, 0, 0, 0],
            CalibrationDirection.POSITIVE_Z: [0, 0, move_distance, 0, 0, 0],
            CalibrationDirection.NEGATIVE_Z: [0, 0, -move_distance, 0, 0, 0],
        }
        return direction_vectors[direction]

    def _calculate_approach_pose(self, target: List[float], direction: CalibrationDirection) -> List[float]:
        """Calculate the approach pose based on target and direction"""
        approach_pose = target.copy()
        offset = self.config.approach_distance

        direction_offsets = {
            CalibrationDirection.POSITIVE_X: [-offset, 0, 0],
            CalibrationDirection.NEGATIVE_X: [offset, 0, 0],
            CalibrationDirection.POSITIVE_Y: [0, -offset, 0],
            CalibrationDirection.NEGATIVE_Y: [0, offset, 0],
            CalibrationDirection.POSITIVE_Z: [0, 0, -offset],
            CalibrationDirection.NEGATIVE_Z: [0, 0, offset],
        }

        pos_offset = direction_offsets[direction]
        approach_pose[0] += pos_offset[0]
        approach_pose[1] += pos_offset[1]
        approach_pose[2] += pos_offset[2]

        return approach_pose

    def _calculate_retreat_pose(self, contact_pose: List[float], direction: CalibrationDirection) -> List[float]:
        """Calculate retreat pose after contact"""
        retreat_pose = contact_pose.copy()
        offset = self.config.retreat_distance

        direction_offsets = {
            CalibrationDirection.POSITIVE_X: [-offset, 0, 0],
            CalibrationDirection.NEGATIVE_X: [offset, 0, 0],
            CalibrationDirection.POSITIVE_Y: [0, -offset, 0],
            CalibrationDirection.NEGATIVE_Y: [0, offset, 0],
            CalibrationDirection.POSITIVE_Z: [0, 0, -offset],
            CalibrationDirection.NEGATIVE_Z: [0, 0, offset],
        }

        pos_offset = direction_offsets[direction]
        retreat_pose[0] += pos_offset[0]
        retreat_pose[1] += pos_offset[1]
        retreat_pose[2] += pos_offset[2]

        return retreat_pose

    def _calculate_calibrated_position(
        self, contact_positions: Dict[CalibrationDirection, List[float]], approximate_target: List[float]
    ) -> List[float]:
        """Calculate final calibrated position from multiple contact points"""
        if len(contact_positions) < 2:
            return approximate_target

        # Extract position components (x, y, z) from contact positions
        positions = []
        for direction, pose in contact_positions.items():
            positions.append(pose[:3])

        # Calculate average position - this gives us the calibrated location
        avg_position = np.mean(positions, axis=0).tolist()

        # Keep orientation from approximate target
        calibrated_pose = avg_position + approximate_target[3:]

        return calibrated_pose

    def get_current_pose(self) -> List[float]:
        """Get current TCP pose"""
        return self.rtde_r.getActualTCPPose()

    def get_current_forces(self) -> List[float]:
        """Get current TCP forces"""
        return self.rtde_r.getActualTCPForce()

    def validate_calibration(self, calibrated_position: List[float]) -> bool:
        """Validate calibrated position with a quick contact test"""
        try:
            # Move slightly above calibrated position
            validation_pose = calibrated_position.copy()
            validation_pose[2] += 0.01  # 10mm above

            self.rtde_c.moveL(validation_pose, self.config.approach_speed, self.config.acceleration)

            # Quick downward contact test
            contact_direction = [0, 0, -0.015, 0, 0, 0]  # 15mm downward
            contact_detected = self.rtde_c.moveUntilContact(
                xd=contact_direction, direction=contact_direction, acceleration=self.config.acceleration
            )

            self.logger.info(f"🔍 Validation result: {'✅ Contact detected' if contact_detected else '❌ No contact'}")
            return contact_detected

        except Exception as e:
            self.logger.error(f"💥 Validation failed: {e}")
            return False

    def cleanup(self):
        """Clean up RTDE connections"""
        try:
            if self.rtde_c:
                self.rtde_c.disconnect()
            if self.rtde_r:
                self.rtde_r.disconnect()
            self.logger.info("🧹 RTDE connections cleaned up")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")


# Easy integration function
def create_ur_calibrator(
    robot_ip: str, approach_speed: float = 0.008, approach_distance: float = 0.025
) -> URCalibrationRTDE:
    """
    Create a UR calibrator with sensible defaults

    Args:
        robot_ip: IP address of UR robot
        approach_speed: Speed for approach movements (m/s)
        approach_distance: Distance to approach from (m)

    Returns:
        Configured URCalibrationRTDE instance
    """
    config = CalibrationConfig(
        approach_speed=approach_speed, approach_distance=approach_distance, retreat_distance=0.003, acceleration=0.1
    )

    return URCalibrationRTDE(robot_ip, config)


# Usage example
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    print("🤖 UR Robot Calibration System with ur_rtde")
    print("=" * 50)

    if not RTDE_AVAILABLE:
        print("❌ ur_rtde not available. Please install it first!")
        print("\nTry these installation methods:")
        print("1. conda install -c conda-forge ur_rtde")
        print("2. pip install ur_rtde==1.5.7")
        print("3. Use Docker solution")
        exit(1)

    print("✅ ur_rtde is available and ready!")
    print("\nExample usage:")
    print("""
# Create calibrator
robot_ip = "192.168.1.100"
calibrator = create_ur_calibrator(robot_ip)

# Define calibration parameters
target = [0.5, 0.2, 0.3, 0, 0, 0]  # Approximate target position
directions = [CalibrationDirection.POSITIVE_X, CalibrationDirection.NEGATIVE_X]
tcp_offset = [0, 0, 0.05, 0, 0, 0]  # 50mm calibration probe

# Perform calibration
result = calibrator.calibrate_target_location(
    approximate_target=target,
    approach_directions=directions,
    tcp_offset=tcp_offset
)

# Check results
if result.success:
    print(f"✅ Calibrated position: {result.calibrated_position}")

    # Validate
    if calibrator.validate_calibration(result.calibrated_position):
        print("✅ Calibration validated!")
    else:
        print("⚠️ Validation failed")
else:
    print(f"❌ Calibration failed: {result.errors}")

# Clean up
calibrator.cleanup()
""")
