"""REST-based client for the Liconic"""
import json, time
from argparse import ArgumentParser, Namespace
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.responses import JSONResponse

from ur_driver.ur_driver import UR

from wei.core.data_classes import (
    ModuleAbout,
    ModuleAction,
    ModuleStatus,
    StepResponse,
    StepStatus,
)
from wei.helpers import extract_version


def parse_args() -> Namespace:
    """Parses CLI args for the REST server"""
    parser = ArgumentParser()
    parser.add_argument("--name", type=str, default="ur_module", help="Module name")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host IP/Domain Name")
    parser.add_argument("--port", type=str, default="3011", help="Port for REST API")
    parser.add_argument("--ur_ip", type=str, default="164.54.116.129", help="IP address of the UR robot")
    return parser.parse_args()

global ur, state

@asynccontextmanager
async def lifespan(app: FastAPI):
    global ur, state
    """Initial run function for the app, initializes the state
        Parameters
        ----------
        app : FastApi
           The REST API app being initialized

        Returns
        -------
        None"""
    try:
        # Do any instrument configuration here
        state = ModuleStatus.IDLE
        ur = UR(args.ur_ip)
    except Exception as err:
        print(err)
        state = ModuleStatus.ERROR

    # Yield control to the application
    yield

    # Do any cleanup here
    pass


app = FastAPI(
    lifespan=lifespan,
)


@app.get("/state")
def get_state():
    """Returns the current state of the module"""
    global ur, state

    ur.ur_dashboard.get_overall_robot_status()

    if "NORMAL" not in ur.ur_dashboard.safety_status: 
        state = ModuleStatus.ERROR
    elif ur.get_movement_state() == "BUSY":
        state = ModuleStatus.BUSY
    else:
        state = ModuleStatus.IDLE

    return JSONResponse(content={"State": state})


@app.get("/about")
async def about():
    """Returns a description of the actions and resources the module supports"""
    global state
    description = {
        'name': args.name,
        'type': 'ur_arm',
        'actions':
        {
            'status': state,
            'pick_tool':'home, tool_loc, docking_axis, payload, tool_name',
            'place_tool':'home, tool_loc, docking_axis, payload, tool_name',
            'gripper_transfer':'home, source, target, source_approach_axis, target_approach_axis, source_approach_distance, target_approach_distance, gripper_open, gripper_close',
      
        }
    }
    return JSONResponse(content={"About": description})


@app.get("/resources")
async def resources():
    """Returns the current resources available to the module"""
    global state
    return JSONResponse(content={"Resources": "TEST"})


@app.post("/action")
def do_action(
    action_handle: str,  # The action to be performed
    action_vars: str,  # Any arguments necessary to run that action
) -> StepResponse:
    
    global ur, state
    step_response = StepResponse(action_response=StepStatus.IDLE)
    
    if state == ModuleStatus.BUSY:
        step_response.action_response=StepStatus.FAILED
        step_response.action_log="Module is busy"
    else:
        try:
            state = ModuleStatus.BUSY
            action_vars = json.loads(action_vars)

            if action_handle == "gripper_transfer":
                home = action_vars.get("home", None)
                source = action_vars.get("source", None)
                target = action_vars.get("target", None)
                source_approach_axis = action_vars.get("source_approach_axis", None)
                target_approach_axis = action_vars.get("target_approach_axis", None)
                source_approach_distance = action_vars.get("source_approach_distance", None)
                target_approach_distance = action_vars.get("target_approach_distance", None)
                gripper_open = action_vars.get("gripper_open", None)
                gripper_close = action_vars.get("gripper_close", None)

                if not source or target or home: #Return Fail
                    pass

                ur.gripper_transfer(home = home, 
                                    source = source, 
                                    target = target, 
                                    source_approach_distance = source_approach_distance,
                                    target_approach_distance = target_approach_distance,
                                    source_approach_axis=source_approach_axis, 
                                    target_approach_axis=target_approach_axis, 
                                    gripper_open = gripper_open, 
                                    gripper_close = gripper_close
                                    )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Gripper transfer from {source} to {target}",
                )
            
            elif action_handle == "pick_tool":
                home = action_vars.get("home", None)
                tool_loc = action_vars.get("tool_loc", None)
                payload = action_vars.get("payload", None)
                docking_axis = action_vars.get("docking_axis", None)
                tool_name = action_vars.get("tool_name", None)

                if not home or tool_loc: #Return Fail
                    pass

                ur.pick_tool(home = home, 
                            tool_loc = tool_loc, 
                            docking_axis = docking_axis, 
                            payload=payload, 
                            tool_name=tool_name
                            )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Pick tool {tool_name} from {tool_loc}",
                )
            
            elif action_handle == "place_tool":
                home = action_vars.get("home", None)
                tool_loc = action_vars.get("tool_loc", None)
                docking_axis = action_vars.get("docking_axis", None)
                tool_name = action_vars.get("tool_name", None)

                if not home or tool_loc: #Return Fail
                    pass

                ur.place_tool(home = home, 
                            tool_loc = tool_loc, 
                            docking_axis = docking_axis, 
                            tool_name=tool_name
                            )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Place tool {tool_name} from {tool_loc}",
                )
            
            elif action_handle == "gripper_screw_transfer":
                home = action_vars.get("home", None)
                screwdriver_loc = action_vars.get("screwdriver_loc", None)
                screw_loc = action_vars.get("screw_loc", None)
                screw_time = action_vars.get("screw_time", None)
                target = action_vars.get("target", None)
                gripper_open = action_vars.get("gripper_open", None)
                gripper_close = action_vars.get("gripper_close", None)

                if not home or screwdriver_loc or screw_loc or target: #Return Fail
                    pass

                ur.gripper_screw_transfer(home = home, 
                                        screwdriver_loc = screwdriver_loc, 
                                        screw_loc = screw_loc, 
                                        screw_time = screw_time,
                                        target = target,
                                        gripper_open = gripper_open,
                                        gripper_close = gripper_close
                                        )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Gripper screw transfer to {target}",
                )
            
            elif action_handle == "pipette_transfer":
                home = action_vars.get("home", None)
                tip_loc = action_vars.get("tip_loc", None)
                tip_trash = action_vars.get("tip_trash", None)
                source = action_vars.get("source", None)
                target = action_vars.get("target", None)
                volume = action_vars.get("volume", None)

                if not home or tip_loc or source or target: #Return Fail
                    pass

                ur.pipette_transfer(home = home, 
                                    tip_loc = tip_loc, 
                                    tip_trash = tip_trash, 
                                    source = source,
                                    target = target,
                                    volume = volume,
                                    )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Pipette transfer to {target}, volume {volume}",
                )
            
            elif action_handle == "pick_and_flip_object":
                home = action_vars.get("home", None)
                target = action_vars.get("target", None)
                approach_axis = action_vars.get("approach_axis", None)
                target_approach_distance = action_vars.get("target_approach_distance", None)
                gripper_open = action_vars.get("gripper_open", None)
                gripper_close = action_vars.get("gripper_close", None)

                if not home or target: #Return Fail
                    pass

                ur.pick_and_flip_object(home = home, 
                                        target = target, 
                                        approach_axis = approach_axis,
                                        target_approach_distance = target_approach_distance,
                                        gripper_open = gripper_open,
                                        gripper_close = gripper_close
                                        )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Pick and flip object at {target}",
                )
            
            elif action_handle == "remove_cap":
                home = action_vars.get("home", None)
                source = action_vars.get("source", None)
                target = action_vars.get("target", None)
                gripper_open = action_vars.get("gripper_open", None)
                gripper_close = action_vars.get("gripper_close", None)

                if not home or source or target: #Return Fail
                    pass

                ur.remove_cap(home = home, 
                            source = source,
                            target = target, 
                            gripper_open = gripper_open,
                            gripper_close = gripper_close
                            )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Remove cap at {target}",
                )
            
            elif action_handle == "place_cap":
                home = action_vars.get("home", None)
                source = action_vars.get("source", None)
                target = action_vars.get("target", None)
                gripper_open = action_vars.get("gripper_open", None)
                gripper_close = action_vars.get("gripper_close", None)

                if not home or source or target: #Return Fail
                    pass

                ur.place_cap(home = home, 
                            source = source,
                            target = target, 
                            gripper_open = gripper_open,
                            gripper_close = gripper_close
                            )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Place cap at {target}",
                )
            
            elif action_handle == "run_urp_program":
                transfer_file_path = action_vars.get("transfer_file_path", None)
                program_name = action_vars.get("program_name", None)

                if not program_name: #Return Fail
                    pass

                ur.run_urp_program(transfer_file_path = transfer_file_path, 
                            program_name = program_name,
                            )

                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.SUCCEEDED,
                    action_msg="",
                    action_log=f"Run rup program {program_name}",
                )
            
            else:
                state = ModuleStatus.IDLE
                return StepResponse(
                    action_response=StepStatus.FAILED,
                    action_msg="False",
                    action_log=f"Action {action_handle} not supported",
                )
        except Exception as e:
            print(str(e))
            state = ModuleStatus.ERROR
            step_response.action_response=StepStatus.FAILED
            step_response.action_log=str(e)
        else:
            state = ModuleStatus.IDLE
    return step_response

if __name__ == "__main__":
    import uvicorn

    args = parse_args()
    
    uvicorn.run(
        "ur_rest_node:app",
        host=args.host,
        port=int(args.port),
        reload=True,
    )