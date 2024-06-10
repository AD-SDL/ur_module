"""REST-based node for UR robots"""

import datetime
import traceback
from typing import List

from fastapi.datastructures import State
from fastapi.responses import JSONResponse
from typing_extensions import Annotated
from ur_driver.ur import UR
from wei.modules.rest_module import RESTModule
from wei.types.module_types import ModuleStatus
from wei.types.step_types import ActionRequest, StepResponse, StepStatus

rest_module = RESTModule(
    name="ur_node",
    version="0.0.1",
    description="A node to control the ur plate moving robot",
    model="ur",
)
rest_module.arg_parser.add_argument(
    "--ur_ip", type=str, help="IP address of the UR robot", default="164.54.116.129"
)


@rest_module.startup()
def ur_startup(state: State):
    """UR startup handler."""
    try:
        state.ur = UR(state.ur_ip)
        state.status = ModuleStatus.IDLE
    except Exception:
        state.status = ModuleStatus.ERROR
        traceback.print_exc()
    else:
        print("UR online")
    yield

    # Do any cleanup here
    pass


def check_state(state: State):
    """Gets robot status by checking robot dashboard status messages."""
    state.ur.ur_dashboard.get_overall_robot_status()
    if "NORMAL" not in state.ur.ur_dashboard.safety_status:
        state.status = ModuleStatus.ERROR
    elif state.ur.get_movement_state() == "BUSY":
        state.status = ModuleStatus.BUSY
    else:
        state.status = ModuleStatus.IDLE


@rest_module.state_handler()
def state(state: State):
    """Returns the current state of the UR module"""

    if (
        not (state.status == ModuleStatus.BUSY)
        or (state.status == ModuleStatus.ERROR)
        or (
            state.action_start
            and (
                datetime.datetime.now() - state.action_start > datetime.timedelta(0, 2)
            )
        )
    ):
        check_state(state)
    return JSONResponse(content={"State": state})


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

    if not source or target or home:  # Return Fail
        return StepResponse(
            StepStatus.FAILED, "", "Source, target and home locations must be provided"
        )

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

    return StepResponse.step_succeeded(
        "Gripper transfer completed from {source} to {target}"
    )


@rest_module.action(
    name="pick_tool",
    description="Picks up a tool using the provided tool location",
)
def pick_tool(
    state: State,
    action: ActionRequest,
    home: Annotated[List[float], "Home location"],
    tool_loc: Annotated[List[float], "Home location"],
    docking_axis: Annotated[str, "Docking axis, (X/Y/Z)"],
    payload: Annotated[float, "Tool payload"],
    tool_name: Annotated[str, "Tool name)"],
) -> StepResponse:
    """Pick a tool with the UR"""

    if not tool_loc or not home:  # Return Fail
        return StepResponse(
            StepStatus.FAILED, "", "Source, target and home locations must be provided"
        )

    # state.ur.gripper_transfer(
    #     home=home,
    #     source=source,
    #     target=target,
    #     source_approach_distance=source_approach_distance,
    #     target_approach_distance=target_approach_distance,
    #     source_approach_axis=source_approach_axis,
    #     target_approach_axis=target_approach_axis,
    #     gripper_open=gripper_open,
    #     gripper_close=gripper_close,
    # )

    return StepResponse(StepStatus.SUCCEEDED, "plate moved", None)


rest_module.start()
