"""REST-based node for UR robots"""

import json
from pathlib import Path
from typing import List

from fastapi.datastructures import State
from typing_extensions import Annotated
from ur_driver.ur import UR
from wei.modules.rest_module import RESTModule
from wei.types.module_types import ModuleState, ModuleStatus
from wei.types.step_types import ActionRequest, StepResponse, StepStatus
from wei.utils import extract_version

rest_module = RESTModule(
    name="ur_node",
    version=extract_version(Path(__file__).parent.parent / "pyproject.toml"),
    description="A node to control the ur plate moving robot",
    model="ur",
)
rest_module.arg_parser.add_argument(
    "--ur_ip",
    type=str,
    default="164.54.116.129",
    help="Hostname or IP address to connect to UR",
)


@rest_module.startup()
def ur_startup(state: State):
    """UR startup handler."""
    state.ur = None
    state.ur = UR(hostname=state.ur_ip)
    print("UR online")


@rest_module.shutdown()
def ur_shutdown(state: State):
    """UR shutdown handler."""
    state.ur.ur_connection.disconnect_ur()
    print("UR offline")


@rest_module.state_handler()
def state(state: State):
    """Returns the current state of the UR module"""
    if state.status not in [ModuleStatus.ERROR, ModuleStatus.INIT, None]:
        # * Gets robot status by checking robot dashboard status messages.
        state.ur.ur_dashboard.get_overall_robot_status()
        if "NORMAL" not in state.ur.ur_dashboard.safety_status:
            state.status = ModuleStatus.ERROR
        elif state.ur.get_movement_state() == "BUSY":
            state.status = ModuleStatus.BUSY
        else:
            state.status = ModuleStatus.IDLE
    return ModuleState(status=state.status, error="")


@rest_module.action()
def movej(
    state: State,
    action: ActionRequest,
    joints: Annotated[List[float], "Joint positions to move to"],
    a: Annotated[float, "Acceleration"] = 0.6,
    v: Annotated[float, "Velocity"] = 0.6,
) -> StepResponse:
    """Move the robot to a joint position"""
    joints = json.loads(joints)
    print(joints)
    state.ur.ur_connection.movej(joints, a, v)
    return StepResponse.step_succeeded()


@rest_module.action()
def toggle_gripper(
    state: State,
    action: ActionRequest,
    open: Annotated[bool, "Open?"] = False,
    close: Annotated[bool, "Close?"] = False,
) -> StepResponse:
    """Open or close the robot gripper."""
    if open:
        state.ur.gripper.open_gripper()
        print("POS: ", state.ur.gripper.gripper.get_current_position())
    if close:
        state.ur.gripper.close_gripper()
        print("POS: ", state.ur.gripper.gripper.get_current_position())
    return StepResponse.step_succeeded()


@rest_module.action(
    name="gripper_transfer",
    description="Execute a transfer in between source and target locations using Robotiq grippers",
)
def gripper_transfer(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    source: Annotated[List[float], "Location to transfer sample from"],
    target: Annotated[List[float], "Location to transfer sample to"],
    source_approach_axis: Annotated[str, "Source location approach axis, (X/Y/Z)"],
    target_approach_axis: Annotated[str, "Source location approach axis, (X/Y/Z)"],
    source_approach_distance: Annotated[float, "Approach distance in meters"],
    target_approach_distance: Annotated[float, "Approach distance in meters"],
    gripper_open: Annotated[int, "Set a max value for the gripper open state"],
    gripper_close: Annotated[int, "Set a min value for the gripper close state"],
) -> StepResponse:
    """Make a transfer using the finger gripper. This function uses linear motions to perform the pick and place movements."""

    if not source or not target or not home:  # Return Fail
        return StepResponse(StepStatus.FAILED, error="Source, target and home locations must be provided")

    state.ur.gripper_transfer(
        home=home,
        source=source,
        target=target,
        source_approach_distance=source_approach_distance,
        target_approach_distance=target_approach_distance,
        source_approach_axis=source_approach_axis,
        target_approach_axis=target_approach_axis,
        gripper_open=gripper_open,
        gripper_close=gripper_close,
    )
    return StepResponse.step_succeeded()


@rest_module.action(
    name="pick_tool",
    description="Picks up a tool using the provided tool location",
)
def pick_tool(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    tool_loc: Annotated[List[float], "Tool location"],
    docking_axis: Annotated[str, "Docking axis, (X/Y/Z)"],
    payload: Annotated[float, "Tool payload"],
    tool_name: Annotated[str, "Tool name)"],
) -> StepResponse:
    """Pick a tool with the UR"""

    if not tool_loc or not home:  # Return Fail
        return StepResponse(StepStatus.FAILED, error="tool_loc and home locations must be provided")

    state.ur.pick_tool(
        home=home,
        tool_loc=tool_loc,
        docking_axis=docking_axis,
        payload=payload,
        tool_name=tool_name,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="Place_tool", description="Places the attached tool back to the provided tool docking location"
)
def place_tool(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    tool_docking: Annotated[List[float], "Tool docking location"],
    docking_axis: Annotated[str, "Docking axis, (X/Y/Z)"],
    tool_name: Annotated[str, "Tool name)"],
) -> StepResponse:
    """Place a tool with the UR"""

    state.ur.place_tool(
        home=home,
        tool_loc=tool_docking,
        docking_axis=docking_axis,
        tool_name=tool_name,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="gripper_screw_transfer",
    description="Performs a screw transfer using the Robotiq gripper and custom screwdriving bits",
)
def gripper_screw_transfer(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    screwdriver_loc: Annotated[List[float], "Screwdriver location"],
    screw_loc: Annotated[List[float], "Screw location"],
    screw_time: Annotated[int, "Srew time in seconds"],
    target: Annotated[List[float], "Location where the srewdriving will be performed"],
    gripper_open: Annotated[int, "Set a max value for the gripper open state"],
    gripper_close: Annotated[int, "Set a min value for the gripper close state"],
) -> StepResponse:
    """Make a screwdriving transfer using Robotiq gripper and custom screwdriving bits with UR"""

    if not home or not screwdriver_loc or not screw_loc or not target:
        return StepResponse(
            StepStatus.FAILED,
            "",
            "screwdriver_loc, screw_loc and home locations must be provided",
        )
    state.ur.gripper_screw_transfer(
        home=home,
        screwdriver_loc=screwdriver_loc,
        screw_loc=screw_loc,
        screw_time=screw_time,
        target=target,
        gripper_open=gripper_open,
        gripper_close=gripper_close,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="pipette_transfer",
    description="Make a pipette transfer to transfer sample liquids in between two locations",
)
def pipette_transfer(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    source: Annotated[List[float], "Initial location of the sample"],
    target: Annotated[List[float], "Target location of the sample"],
    tip_loc=Annotated[List[float], "New tip location"],
    tip_trash=Annotated[List[float], "Tip trash location"],
    volume=Annotated[float, "Set a volume in micro liters"],
) -> StepResponse:
    """Make a pipette transfer for the defined volume with UR"""

    state._state.pipette_transfer(
        home=home,
        tip_loc=tip_loc,
        tip_trash=tip_trash,
        source=source,
        target=target,
        volume=volume,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="pick_and_flip_object",
    description="Picks and flips an object 180 degrees",
)
def pick_and_flip_object(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    target: Annotated[List[float], "Location of the object"],
    approach_axis: Annotated[str, "Approach axis, (X/Y/Z)"],
    target_approach_distance: Annotated[float, "Approach distance in meters"],
    gripper_open: Annotated[int, "Set a max value for the gripper open state"],
    gripper_close: Annotated[int, "Set a min value for the gripper close state"],
) -> StepResponse:
    """Picks and flips an object 180 degrees with UR"""

    state.ur.pick_and_flip_object(
        home=home,
        target=target,
        approach_axis=approach_axis,
        target_approach_distance=target_approach_distance,
        gripper_open=gripper_open,
        gripper_close=gripper_close,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="remove_cap",
    description="Removes caps from sample vials",
)
def remove_cap(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    source: Annotated[List[float], "Location of the vial cap"],
    target: Annotated[
        List[float],
        "Location of where the cap will be placed after it is removed from the vail",
    ],
    gripper_open: Annotated[int, "Set a max value for the gripper open state"],
    gripper_close: Annotated[int, "Set a min value for the gripper close state"],
) -> StepResponse:
    """Remove caps from sample vials with UR"""

    state.ur.remove_cap(
        home=home,
        source=source,
        target=target,
        gripper_open=gripper_open,
        gripper_close=gripper_close,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="place_cap",
    description="Places caps back to sample vials",
)
def place_cap(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    source: Annotated[List[float], "Vail cap initial location"],
    target: Annotated[List[float], "The vail location where the cap will installed"],
    gripper_open: Annotated[int, "Set a max value for the gripper open state"],
    gripper_close: Annotated[int, "Set a min value for the gripper close state"],
) -> StepResponse:
    """Places caps back to sample vials with UR"""

    state.ur.place_cap(
        home=home,
        source=source,
        target=target,
        gripper_open=gripper_open,
        gripper_close=gripper_close,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="run_urp_program",
    description="Runs a URP program on the UR",
)
def run_urp_program(
    state: State,
    action: ActionRequest,
    transfer_file_path=Annotated[str, "Transfer file path"],
    program_name=Annotated[str, "Program name"],
) -> StepResponse:
    """Run an URP program on the UR"""

    state.ur.run_urp_program(
        transfer_file_path=transfer_file_path,
        program_name=program_name,
    )

    return StepResponse.step_succeeded()


@rest_module.action(
    name="set_digital_io",
    description="Sets a channel IO output on the UR",
)
def set_digital_io(
    state: State,
    action: ActionRequest,
    channel=Annotated[int, "Channel number"],
    value=Annotated[bool, "True/False"],
) -> StepResponse:
    """Sets a channel IO output on the UR"""

    state.ur.set_digital_io(channel=channel, value=value)

    return StepResponse.step_succeeded()


if __name__ == "__main__":
    rest_module.start()
