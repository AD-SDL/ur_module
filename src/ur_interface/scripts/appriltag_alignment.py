"""AprilTag Alignment Module - Complete and Self-Contained

This module contains ALL AprilTag detection and alignment logic for 12IDB beamline.
Includes move_toward_camera implementation - no need to add to UR class!

The UR class only needs to call the two alignment functions.

Dependencies:
- pip install pupil-apriltags opencv-python numpy math3d
"""

import logging
import math
import time
from typing import Any, Callable, Dict, Optional

import math3d as m3d
import numpy as np

# Import force_calibration for bump operations
from ur_interface.scripts import force_calibration

# AprilTag detection
try:
    import cv2
    from pupil_apriltags import Detector

    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("Warning: pupil-apriltags or opencv-python not installed")
    print("Install with: pip install pupil-apriltags opencv-python")


# =============================================================================
# Configuration - All settings contained in this module
# =============================================================================

# AprilTag sizes (meters)
APRILTAG_SIZES = {
    "heater": 0.0075,  # 7.5mm tag
    "standard": 0.015,  # 15mm tag
}

# Waypoints for camera positioning (from robot12idb.py)
WAYPOINT_CAMERA_HEATER = [0.06742913, -0.37484296, -0.01921846, 1.59462232, -1.56894371, -0.90682349]

WAYPOINT_CAMERA_STANDARD = [-0.09136841, -0.37766972, 0.15913981, -1.92979616, 1.92055087, -0.51508574]

# Camera mounting angles (from robUR.py)
CAMERA_TILT_ANGLE = math.pi / 3  # 60° tilt
CAMERA_NORTH_ANGLE = -math.pi / 6  # -30° for north direction


# =============================================================================
# AprilTag Camera Class
# =============================================================================


class AprilTagCamera:
    """Camera interface for AprilTag detection

    This class handles all camera operations and AprilTag detection.
    """

    def __init__(self, camera_index: int = 0, tag_family: str = "tag36h11"):
        """Initialize AprilTag camera

        Args:
            camera_index: Camera device index (default: 0)
            tag_family: AprilTag family (default: 'tag36h11')
        """
        if not APRILTAG_AVAILABLE:
            raise RuntimeError("AprilTag libraries not available")

        self.camera_index = camera_index
        self.cap = None
        self.detector = Detector(families=tag_family)
        self.logger = logging.getLogger("AprilTagCamera")

        # Image storage
        self.image = None  # Last captured color image
        self.decoded = None  # Last AprilTag detection result

        # Camera calibration
        # IMPORTANT: Calibrate this for your specific camera!
        self.camera_f = 600.0  # Focal length in pixels

    def connect(self):
        """Connect to camera"""
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.camera_index}")
        self.logger.info(f"Camera {self.camera_index} connected")

    def disconnect(self):
        """Disconnect camera"""
        if self.cap:
            self.cap.release()
            self.logger.info("Camera disconnected")

    def capture(self) -> np.ndarray:
        """Capture image from camera"""
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError("Camera not connected")

        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to capture image")

        self.image = frame
        return self.image

    def decodeAT(self):
        """Detect AprilTags in current image"""
        if self.image is None:
            return None

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        if detections:
            self.decoded = detections[0]
            return self.decoded

        return None

    def getATdistance(self, detection, tag_size: float = 0.0075) -> float:
        """Calculate distance to AprilTag"""
        if detection is None:
            return None

        tag_width_pixels = np.linalg.norm(detection.corners[0] - detection.corners[1])

        distance = (tag_size * self.camera_f) / tag_width_pixels
        return distance


# =============================================================================
# Camera Coordinate Movement - EXACT robUR.py Implementation
# =============================================================================


def _get_camera_vector(ur_connection):
    """Get camera coordinate system vectors - EXACT robUR.py logic

    Calculates camera coordinate system based on tool orientation.
    Camera mounting angles: 60° tilt, -30° north (from robUR.py)

    Args:
        ur_connection: UR robot connection

    Returns:
        cameravector: Vector pointing in camera's forward direction
        cameravector_north: Vector pointing up in camera view
        cameravector_east: Vector pointing right in camera view
    """
    # Get current tool orientation
    current_pose_list = ur_connection.getl()
    pose = m3d.Transform.new_from_xyzvec(current_pose_list)

    # Get east vector (X axis of tool)
    cameravector_east = pose.orient.get_vec_x()

    # Calculate camera pointing direction (rotated 60°)
    v = pose.orient.copy()
    v.rotate_b(cameravector_east, CAMERA_TILT_ANGLE)
    cameravector = v.get_vec_y()

    # Calculate north vector (rotated -30°)
    v = pose.orient.copy()
    v.rotate_b(cameravector_east, CAMERA_NORTH_ANGLE)
    cameravector_north = v.get_vec_y()

    return cameravector, cameravector_north, -cameravector_east


def _move_toward_camera(ur_connection, distance, north=0.0, east=0.0, acc=0.5, vel=0.5):
    """Move in camera coordinate system - EXACT robUR.py logic

    Moves robot in camera coordinates rather than robot base coordinates.
    Essential for AprilTag centering where offsets are in camera frame.

    Args:
        ur_connection: UR robot connection
        distance: Distance along camera pointing direction (meters)
        north: Distance up in camera view (meters)
        east: Distance right in camera view (meters)
        acc: Acceleration
        vel: Velocity
    """
    # Get camera coordinate vectors
    cameravector, northv, eastv = _get_camera_vector(ur_connection)

    # Calculate movement in camera coordinates
    movement_vector = cameravector * distance + eastv * east + northv * north

    # Get current position
    current_pose_list = ur_connection.getl()

    # Add movement to current position
    new_pose = [
        current_pose_list[0] + movement_vector[0],
        current_pose_list[1] + movement_vector[1],
        current_pose_list[2] + movement_vector[2],
        current_pose_list[3],  # Keep orientation
        current_pose_list[4],
        current_pose_list[5],
    ]

    # Move to new position
    # Try set_pose first (urx.Robot standard), fall back to movel
    try:
        new_transform = m3d.Transform.new_from_xyzvec(new_pose)
        ur_connection.set_pose(new_transform, acc=acc, vel=vel, wait=True, command="movej")
    except AttributeError:
        # set_pose not available, use movel
        ur_connection.movel(new_pose, acc=acc, vel=vel)


# =============================================================================
# Internal Helper Functions
# =============================================================================


def _center_aprilTag(camera: AprilTagCamera, ur_connection) -> bool:
    """Center AprilTag in camera view - ORIGINAL LOGIC

    Args:
        camera: AprilTagCamera instance
        ur_connection: UR robot connection

    Returns:
        True if successful, False if failed
    """
    logger = logging.getLogger("center_aprilTag")

    if camera.image is None:
        logger.error("No image captured")
        return False

    r = camera.decoded
    if r is None:
        logger.error("No AprilTag in camera")
        return False

    # Get image dimensions
    h, w, _ = camera.image.shape

    # Get AprilTag center position in pixels
    QRpos = r.center

    # Calculate distance to tag
    tag_size = APRILTAG_SIZES["heater"]
    QRdist = camera.getATdistance(r, tag_size)

    # Calculate pixel offset from image center (ORIGINAL FORMULA)
    dx = w / 2 - QRpos[0]
    dy = h / 2 - QRpos[1]

    # Convert pixel offset to physical distance (ORIGINAL FORMULA)
    dX = -dx / camera.camera_f * QRdist
    dY = dy / camera.camera_f * QRdist

    # Move robot in camera coordinate system
    _move_toward_camera(ur_connection, distance=0, north=dY, east=dX, acc=0.5, vel=0.5)

    return True


def _center_camera2apriltag(camera: AprilTagCamera, ur_connection, max_trials: int = 10) -> bool:
    """Center camera on AprilTag with iteration - ORIGINAL LOGIC

    Args:
        camera: AprilTagCamera instance
        ur_connection: UR robot connection
        max_trials: Maximum number of attempts

    Returns:
        True if AprilTag found and centered, False otherwise
    """
    logger = logging.getLogger("center_camera2apriltag")

    trial = 0
    done = False

    while trial < max_trials:
        # Capture image
        _ = camera.capture()

        # Detect AprilTag
        r = camera.decodeAT()

        if r is not None:
            done = True
            break

        trial += 1
        time.sleep(0.2)

    if done:
        # Center on the detected tag
        _center_aprilTag(camera, ur_connection)
        return done

    logger.error(f"Cannot find AprilTag after {max_trials} attempts")
    return False


# =============================================================================
# Public API - Major Two Methods
# =============================================================================


def auto_align_12idb_remote_heater(
    ur_connection,
    camera: AprilTagCamera,
    home: Optional[Callable] = None,
    home_position: Optional[list] = None,
    test_run: bool = True,
    gripper: Any = None,
) -> Dict:
    """Automatic alignment to 12IDB remote heater - ORIGINAL LOGIC

    This is the EXACT sequence from robot12idb.py.
    All configuration and camera movement logic is internal to this module.

    Args:
        ur_connection: UR robot connection object with methods:
                      - getl(), getj(), movel(), movej()
        camera: AprilTagCamera instance (already connected)
        gripper_close: Optional function to close gripper
        gripper_open: Optional function to open gripper
        home: Optional function to move to home
        home_position: Optional home position
        test_run: If True, runs test pickup/dropoff after alignment

    Returns:
        Dictionary with status, aligned_position, and tilt_compensation
    """
    logger = logging.getLogger("auto_align_heater")

    try:
        logger.info("=" * 60)
        logger.info("Starting automatic heater alignment")
        logger.info("=" * 60)

        # Configuration (ORIGINAL VALUES)
        dist2ATtag = 0.3
        barlength = 0.11
        gripper_width = 0.02
        v_standoff = 0.02

        # Step 1: Move to default
        logger.info("Step 1: Moving to default position")
        if home and home_position:
            home(home_position)

        # Step 2: Move camera to view AprilTag
        logger.info("Step 2: Positioning camera to view AprilTag")
        ur_connection.movel(WAYPOINT_CAMERA_HEATER, acc=0.5, vel=0.5)

        # Step 3: Center on AprilTag
        logger.info("Step 3: Centering on AprilTag")
        ret = _center_camera2apriltag(camera, ur_connection)
        if not ret:
            return {"status": "error", "message": "Failed to find or center AprilTag"}

        logger.info("AprilTag found and centered!")

        # Step 4: Move to calibrated distance
        d = camera.getATdistance(camera.decoded, APRILTAG_SIZES["heater"])
        logger.info(f"Step 4: Relocating to {dist2ATtag}m from tag (currently {d:.3f}m)")

        current_pos = ur_connection.getl()
        current_pos[0] += dist2ATtag - d
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 5: Close gripper
        logger.info("Step 5: Closing gripper")
        gripper.close_gripper()

        # Step 6: Bump to confirm distance
        logger.info("Step 6: Confirming distance by bumping")
        force_calibration.bump(ur_connection=ur_connection, x=-0.2, backoff=0.05, force=40)

        current_pos = ur_connection.getl()
        current_pos[2] += 0.05
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 7: Move to bar center
        logger.info("Step 7: Moving to bar center")
        current_pos = ur_connection.getl()
        current_pos[0] -= barlength / 2 + 0.05 + gripper_width / 2
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 8: Rotate gripper
        logger.info("Step 8: Rotating gripper -90 degrees")
        current_joints = ur_connection.getj()
        current_joints[5] -= math.radians(90)
        ur_connection.movej(current_joints, acc=0.5, vel=0.5)

        # Step 9: Align along beam
        logger.info("Step 9: Aligning along beam")
        current_pos = ur_connection.getl()
        current_pos[1] -= 0.04
        current_pos[2] -= 0.04
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        force_calibration.bump(ur_connection=ur_connection, y=0.1, backoff=0.02, force=40)

        current_pos = ur_connection.getl()
        current_pos[2] += 0.04
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)
        current_pos[1] += 0.032
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 10: Check Z position
        logger.info("Step 10: Checking Z position")
        force_calibration.bump(ur_connection=ur_connection, z=-0.1, backoff=v_standoff, force=40)
        p0 = ur_connection.getl()

        # Step 11: Measure tilt
        logger.info("Step 11: Measuring tilt")
        z_tempdown = v_standoff + 0.005

        current_pos = ur_connection.getl()
        current_pos[1] += 0.015
        current_pos[2] -= z_tempdown
        current_pos[0] += barlength / 2
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        force_calibration.bump(ur_connection=ur_connection, y=-0.01, force=40)
        p1 = ur_connection.getl()

        current_pos = ur_connection.getl()
        current_pos[1] += 0.005
        current_pos[0] -= barlength
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        force_calibration.bump(ur_connection=ur_connection, y=-0.01, force=40)
        p2 = ur_connection.getl()

        # Step 12: Apply tilt compensation
        ang = math.atan((p2[1] - p1[1]) / barlength) * 180 / math.pi
        logger.info(f"Step 12: Tilt = {ang:.2f} degrees, compensating...")

        current_pos = ur_connection.getl()
        current_pos[1] += 0.005
        current_pos[0] += barlength / 2
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        current_joints = ur_connection.getj()
        current_joints[5] += math.radians(ang)
        ur_connection.movej(current_joints, acc=0.5, vel=0.5)

        current_pos = ur_connection.getl()
        current_pos[2] += z_tempdown
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        ur_connection.movel(p0, acc=0.5, vel=0.5)

        # Step 13: Set final position
        logger.info("Step 13: Setting final position")
        gripper.open_gripper()

        current_pos = ur_connection.getl()
        current_pos[2] -= v_standoff + 0.015
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        aligned_position = ur_connection.getl()

        current_pos = aligned_position.copy()
        current_pos[2] += 0.05
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        logger.info("=" * 60)
        logger.info("Alignment complete!")
        logger.info(f"Position: {aligned_position[:3]}")
        logger.info(f"Tilt: {ang:.2f} degrees")
        logger.info("=" * 60)

        # Optional test
        if test_run:
            logger.info("Running test...")
            time.sleep(3)

            ur_connection.movel(aligned_position, acc=0.5, vel=0.5)
            gripper.close_gripper()
            time.sleep(0.5)

            current_pos = aligned_position.copy()
            current_pos[2] += 0.05
            ur_connection.movel(current_pos, acc=0.5, vel=0.5)
            time.sleep(1)

            ur_connection.movel(aligned_position, acc=0.5, vel=0.5)
            gripper.open_gripper()
            time.sleep(0.5)

            current_pos[2] += 0.05
            ur_connection.movel(current_pos, acc=0.5, vel=0.5)

            logger.info("Test complete!")

        return {
            "status": "success",
            "message": "Heater alignment completed",
            "aligned_position": aligned_position,
            "tilt_compensation": ang,
        }

    except Exception as e:
        logger.error(f"Alignment failed: {e}")
        import traceback

        logger.error(traceback.format_exc())
        return {"status": "error", "message": f"Alignment failed: {str(e)}"}


def auto_align_12idb_standard_holder(
    ur_connection,
    gripper_close: Any = None,
    gripper_open: Any = None,
    home: Optional[Callable] = None,
    home_position: Optional[list] = None,
    test_run: bool = True,
) -> Dict:
    """Automatic alignment to 12IDB standard holder - ORIGINAL LOGIC

    Args:
        ur_connection: UR robot connection object
        gripper_close: Optional function to close gripper
        gripper_open: Optional function to open gripper
        home: Optional function to move to home
        home_position: Optional home position
        test_run: If True, runs test pickup/dropoff

    Returns:
        Dictionary with status and aligned_position
    """
    logger = logging.getLogger("auto_align_standard")

    try:
        logger.info("=" * 60)
        logger.info("Starting standard holder alignment")
        logger.info("=" * 60)

        barlength = 0.11
        gripper_width = 0.023

        # Step 1: Move to default
        logger.info("Step 1: Moving to default")
        if home and home_position:
            home(home_position)

        # Step 2: Move to waypoint
        logger.info("Step 2: Moving to standard waypoint")
        if gripper_open:
            gripper_open()

        ur_connection.movel(WAYPOINT_CAMERA_STANDARD, acc=0.5, vel=0.5)

        # Step 3: Move forward
        current_pos = ur_connection.getl()
        current_pos[0] += 0.04
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 4: Find X
        logger.info("Step 3: Finding X position")
        if gripper_close:
            gripper_close()

        force_calibration.bump(ur_connection=ur_connection, x=-0.1, backoff=0.005, force=40)

        # Step 5: Move up
        current_pos = ur_connection.getl()
        current_pos[2] += 0.03
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 6: Move to center
        logger.info("Step 4: Moving to center")
        current_pos[0] -= gripper_width / 2 + barlength / 2 + 0.005
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        # Step 7: Find Z
        logger.info("Step 5: Finding Z position")
        force_calibration.bump(ur_connection=ur_connection, z=-0.1, backoff=0.005, force=40)

        # Step 8: Set final position
        logger.info("Step 6: Setting final position")
        if gripper_open:
            gripper_open()

        current_pos = ur_connection.getl()
        current_pos[2] -= 0.02 + 0.005
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        aligned_position = ur_connection.getl()

        current_pos = aligned_position.copy()
        current_pos[2] += 0.05
        ur_connection.movel(current_pos, acc=0.5, vel=0.5)

        logger.info("=" * 60)
        logger.info("Alignment complete!")
        logger.info(f"Position: {aligned_position[:3]}")
        logger.info("=" * 60)

        # Optional test
        if test_run:
            logger.info("Running test...")
            time.sleep(1)

            ur_connection.movel(aligned_position, acc=0.5, vel=0.5)
            if gripper_close:
                gripper_close()
            time.sleep(0.5)

            current_pos = aligned_position.copy()
            current_pos[2] += 0.05
            ur_connection.movel(current_pos, acc=0.5, vel=0.5)
            time.sleep(1)

            ur_connection.movel(aligned_position, acc=0.5, vel=0.5)
            if gripper_open:
                gripper_open()
            time.sleep(0.5)

            current_pos[2] += 0.05
            ur_connection.movel(current_pos, acc=0.5, vel=0.5)

            logger.info("Test complete!")

        return {
            "status": "success",
            "message": "Standard holder alignment completed",
            "aligned_position": aligned_position,
        }

    except Exception as e:
        logger.error(f"Alignment failed: {e}")
        import traceback

        logger.error(traceback.format_exc())
        return {"status": "error", "message": f"Alignment failed: {str(e)}"}
