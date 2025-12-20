"""Force Calibration Module - Self-Contained

This module contains all force-based positioning and calibration logic.
Includes the bump() method using checkdistance.script and tool_contact().

All configuration and script loading is contained within this module.
"""

import logging
import os
import time

# =============================================================================
# Load checkdistance.script
# =============================================================================

# Global variable to store loaded script
_CHECKDISTANCE_SCRIPT = None


def _load_checkdistance_script() -> str:
    """Load checkdistance.script automatically

    Tries to find script in common locations relative to this file.

    Returns:
        Script content as string

    Raises:
        FileNotFoundError: If script not found
    """
    global _CHECKDISTANCE_SCRIPT

    if _CHECKDISTANCE_SCRIPT is not None:
        return _CHECKDISTANCE_SCRIPT

    logger = logging.getLogger("force_calibration")

    # Try to find script relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    possible_paths = [
        os.path.join(current_dir, "checkdistance.script"),
        os.path.join(current_dir, "..", "urscripts", "checkdistance.script"),
        os.path.join(current_dir, "urscripts", "checkdistance.script"),
    ]

    for path in possible_paths:
        if os.path.exists(path):
            with open(path, "r") as f:
                _CHECKDISTANCE_SCRIPT = f.read()
            logger.info(f"Loaded checkdistance.script from {path}")
            return _CHECKDISTANCE_SCRIPT

    raise FileNotFoundError("checkdistance.script not found. Please place it in ur_tools/ directory.")


# =============================================================================
# Bump Function - Force-Guided Positioning
# =============================================================================


def bump(
    ur_connection, x: float = 0, y: float = 0, z: float = 0, backoff: float = 0, force: float = 0, wait: bool = True
) -> None:
    """Force-guided positioning using tool contact - EXACT URXE IMPLEMENTATION

    This is the EXACT implementation from urxe/robot.py (lines 98-108).

    Uses checkdistance.script which:
    1. Starts movement in specified direction
    2. Monitors tool_contact() for force threshold
    3. Stops immediately when contact detected
    4. Calculates exact contact point from joint history
    5. Backs off in the movement direction

    Args:
        ur_connection: UR robot connection with methods:
                      - send_program(script)
                      - is_program_running()
        x: Distance to move in X (meters, negative = toward board)
        y: Distance to move in Y (meters)
        z: Distance to move in Z (meters, negative = down)
        backoff: Distance to back off after contact (meters)
        force: Force threshold (steps to retract, default: 0)
        wait: If True, wait for movement to complete

    Example:
        bump(robot.ur_connection, x=-0.2, backoff=0.05, force=40)
        bump(robot.ur_connection, y=0.1, backoff=0.02, force=40)
        bump(robot.ur_connection, z=-0.1, backoff=0.02, force=40)
    """
    logger = logging.getLogger("bump")

    # Load checkdistance.script
    try:
        script = _load_checkdistance_script()
    except FileNotFoundError as e:
        logger.error(str(e))
        raise

    # Replace placeholders in script (EXACT URXE IMPLEMENTATION)
    data = script.replace("__replace__", f"[{x}, {y}, {z}, 0, 0, 0]")
    data = data.replace("__backoff__", f"{backoff}")
    data = data.replace("__rep_force__", f"{force}")

    logger.debug(f"Bump: x={x}, y={y}, z={z}, backoff={backoff}, force={force}")

    # Send program to robot
    ur_connection.send_program(data)

    # Wait for program to start (EXACT URXE IMPLEMENTATION)
    while not ur_connection.is_program_running():
        time.sleep(0.01)

    # Wait for completion if requested (EXACT URXE IMPLEMENTATION)
    if wait:
        while ur_connection.is_program_running():
            time.sleep(0.01)

    logger.debug("Bump complete")


# =============================================================================
# Force Calibration Helper Functions
# =============================================================================


def test_bump_single_axis(
    ur_connection, axis: str, distance: float = -0.05, backoff: float = 0.01, force: float = 40
) -> dict:
    """Test bump on a single axis

    Args:
        ur_connection: UR robot connection
        axis: Axis to test ('x', 'y', or 'z')
        distance: Distance to attempt (meters)
        backoff: Backoff distance (meters)
        force: Force threshold

    Returns:
        Dictionary with test results
    """
    logger = logging.getLogger("test_bump")

    axis = axis.lower()
    if axis not in ["x", "y", "z"]:
        return {"status": "error", "message": f"Invalid axis: {axis}"}

    logger.info(f"Testing bump on {axis.upper()} axis")
    logger.info(f"Distance: {distance}m, Backoff: {backoff}m, Force: {force}")

    try:
        # Get starting position
        start_pos = ur_connection.getl()
        logger.info(f"Start position: {start_pos[:3]}")

        # Perform bump
        kwargs = {axis: distance, "backoff": backoff, "force": force}
        bump(ur_connection, **kwargs)

        # Get final position
        end_pos = ur_connection.getl()
        logger.info(f"End position: {end_pos[:3]}")

        # Calculate actual distance moved
        axis_idx = {"x": 0, "y": 1, "z": 2}[axis]
        actual_distance = end_pos[axis_idx] - start_pos[axis_idx]

        logger.info(f"Actual distance moved: {actual_distance:.4f}m")

        return {
            "status": "success",
            "axis": axis,
            "requested_distance": distance,
            "actual_distance": actual_distance,
            "start_position": start_pos,
            "end_position": end_pos,
            "backoff": backoff,
            "force": force,
        }

    except Exception as e:
        logger.error(f"Bump test failed: {e}")
        import traceback

        logger.error(traceback.format_exc())
        return {"status": "error", "message": f"Bump test failed: {str(e)}"}


def test_bump_all_axes(ur_connection, distance: float = -0.05, backoff: float = 0.01, force: float = 40) -> dict:
    """Test bump on all three axes

    Args:
        ur_connection: UR robot connection
        distance: Distance to attempt (meters)
        backoff: Backoff distance (meters)
        force: Force threshold

    Returns:
        Dictionary with results for all axes
    """
    logger = logging.getLogger("test_bump_all")

    logger.info("=" * 60)
    logger.info("Testing bump on all axes")
    logger.info("=" * 60)

    results = {
        "x": test_bump_single_axis(ur_connection, "x", distance, backoff, force),
        "y": test_bump_single_axis(ur_connection, "y", distance, backoff, force),
        "z": test_bump_single_axis(ur_connection, "z", distance, backoff, force),
    }

    # Summary
    logger.info("=" * 60)
    logger.info("Bump Test Summary")
    logger.info("=" * 60)
    for axis, result in results.items():
        if result["status"] == "success":
            logger.info(
                f"{axis.upper()}: {result['actual_distance']:.4f}m "
                + f"(requested: {result['requested_distance']:.4f}m)"
            )
        else:
            logger.info(f"{axis.upper()}: FAILED - {result['message']}")

    return results


def calibrate_force_threshold(
    ur_connection, axis: str = "z", min_force: float = 10, max_force: float = 100, step: float = 10
) -> dict:
    """Calibrate force threshold by testing different values

    Args:
        ur_connection: UR robot connection
        axis: Axis to test ('x', 'y', or 'z')
        min_force: Minimum force threshold to test
        max_force: Maximum force threshold to test
        step: Step size for force increments

    Returns:
        Dictionary with calibration results
    """
    logger = logging.getLogger("calibrate_force")

    logger.info("=" * 60)
    logger.info(f"Calibrating force threshold on {axis.upper()} axis")
    logger.info(f"Range: {min_force} to {max_force}, step: {step}")
    logger.info("=" * 60)

    results = []

    force = min_force
    while force <= max_force:
        logger.info(f"\nTesting force threshold: {force}")

        result = test_bump_single_axis(ur_connection, axis, distance=-0.05, backoff=0.01, force=force)

        results.append({"force": force, "result": result})

        # Wait between tests
        time.sleep(2)

        force += step

    # Find optimal force
    successful = [r for r in results if r["result"]["status"] == "success"]
    if successful:
        # Recommend the middle value of successful range
        optimal_force = successful[len(successful) // 2]["force"]
        logger.info(f"\nRecommended force threshold: {optimal_force}")
    else:
        optimal_force = None
        logger.warning("\nNo successful force thresholds found")

    return {"status": "success", "axis": axis, "results": results, "optimal_force": optimal_force}


# =============================================================================
# Position Verification
# =============================================================================


def verify_position_accuracy(ur_connection, target_position: list, tolerance: float = 0.001) -> dict:
    """Verify that robot reached target position within tolerance

    Args:
        ur_connection: UR robot connection
        target_position: Target position [x, y, z, rx, ry, rz]
        tolerance: Position tolerance in meters (default: 1mm)

    Returns:
        Dictionary with verification results
    """
    logger = logging.getLogger("verify_position")

    current_pos = ur_connection.getl()

    # Calculate position error
    error = [abs(current_pos[i] - target_position[i]) for i in range(3)]
    max_error = max(error)

    within_tolerance = max_error < tolerance

    logger.info("Position verification:")
    logger.info(f"  Target:  {target_position[:3]}")
    logger.info(f"  Current: {current_pos[:3]}")
    logger.info(f"  Error:   {error}")
    logger.info(f"  Max error: {max_error:.4f}m")
    logger.info(f"  Within tolerance ({tolerance}m): {within_tolerance}")

    return {
        "status": "success" if within_tolerance else "warning",
        "target_position": target_position,
        "current_position": current_pos,
        "error": error,
        "max_error": max_error,
        "tolerance": tolerance,
        "within_tolerance": within_tolerance,
    }


# =============================================================================
# Example Calibration Workflow
# =============================================================================


def run_calibration_workflow(ur_connection) -> dict:
    """Run complete force calibration workflow

    This tests bump on all axes and calibrates force threshold.

    Args:
        ur_connection: UR robot connection

    Returns:
        Dictionary with complete calibration results
    """
    logger = logging.getLogger("calibration_workflow")

    logger.info("=" * 60)
    logger.info("Starting Force Calibration Workflow")
    logger.info("=" * 60)

    # Step 1: Test bump on all axes with default force
    logger.info("\nStep 1: Testing bump on all axes")
    bump_results = test_bump_all_axes(ur_connection, force=40)

    # Step 2: Calibrate force threshold on Z axis
    logger.info("\nStep 2: Calibrating force threshold")
    calibration_results = calibrate_force_threshold(ur_connection, axis="z", min_force=20, max_force=60, step=10)

    logger.info("=" * 60)
    logger.info("Calibration Workflow Complete")
    logger.info("=" * 60)

    return {"status": "success", "bump_test_results": bump_results, "force_calibration": calibration_results}
