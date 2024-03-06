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
    state == ModuleStatus.IDLE
    # if ur.ready: 
    #     state = ModuleStatus.IDLE
    # else:
    #     if ur.has_error: 
    #         state = ModuleStatus.ERROR
    #     else: 
    #         state = ModuleStatus.BUSY
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
            'pick_tool':'home, tool_loc',
            'place_tool':'home, tool_loc',
            'gripper_transfer':'home, source, target, source_approach_axis, target_approach_axis, source_approach_distance, target_approach_distance, gripper_open, gripper_close',
            'get_current_humidity':'',
            'get_target_humidity':'',
            'set_target_humidity':'humidity',
            'begin_shake':'shaker_speed',
            'end_shake':'',
            'load_plate':'stacker, slot',
            'unload_plate':'stacker, slot',
        }
    }
    return JSONResponse(content={"About": description})


@app.get("/resources")
async def resources():
    """Returns the current resources available to the module"""
    global state, module_resources
    return JSONResponse(content={"Resources": module_resources.resources})


@app.post("/action")
def do_action(
    action_handle: str,  # The action to be performed
    action_vars: str,  # Any arguments necessary to run that action
) -> StepResponse:
    global state, ur, module_resources
    step_response = StepResponse(action_response=StepStatus.IDLE)
    if state == ModuleStatus.BUSY:
        step_response.action_response=StepStatus.FAILED
        step_response.action_log="Module is busy"
    else:
        try:
            state = ModuleStatus.BUSY
            action_args = json.loads(action_vars)
            if action_handle == "get_current_temp":
                state = ModuleStatus.IDLE
                step_response.action_response=StepStatus.SUCCEEDED
                step_response.action_msg=str(liconic.climate_controller.current_temperature)
            elif action_handle == "get_target_temp":
                state = ModuleStatus.IDLE
                step_response.action_response=StepStatus.SUCCEEDED
                step_response.action_msg=str(liconic.climate_controller.target_temperature)
            elif action_handle == "set_target_temp":
                try:
                    temp = float(action_args.get('temp', None))
                    liconic.climate_controller.target_temperature = temp
                    state = ModuleStatus.IDLE
                    step_response.action_response=StepStatus.SUCCEEDED
                    step_response.action_msg=f"Set temperature to {temp}"
                except ValueError as val_err:
                    error_msg = "Error: temp argument must be a float"
                    print(error_msg)
                    state = ModuleStatus.IDLE
                    step_response.action_response=StepStatus.FAILED
                    step_response.action_log=error_msg
            elif action_handle == "get_current_humidity":
                state = ModuleStatus.IDLE
                step_response.action_response=StepStatus.SUCCEEDED
                step_response.action_msg=str(liconic.climate_controller.current_humidity)
            elif action_handle == "get_target_humidity":
                state = ModuleStatus.IDLE
                step_response.action_response=StepStatus.SUCCEEDED
                step_response.action_msg=str(liconic.climate_controller.target_humidity)
            elif action_handle == "set_target_humidity":
                try:
                    humidity = float(action_args.get('humidity', None))
                    liconic.climate_controller.target_humidity = humidity
                    state = ModuleStatus.IDLE
                    step_response.action_response=StepStatus.SUCCEEDED
                    step_response.action_msg=f"Set humidity to {humidity}"
                except ValueError as val_err:
                    error_msg = "Error: humidity argument must be a float"
                    print(error_msg)
                    state = ModuleStatus.IDLE
                    step_response.action_response=StepStatus.FAILED
                    step_response.action_log=error_msg
            elif action_handle == "begin_shake":
                try:
                    new_shaker_speed = int(action_args.get('shaker_speed'))
                    if liconic.shaker_controller.shaker_is_active:
                        if not new_shaker_speed == liconic.shaker_controller.shaker_speed:
                            """already shaking but not at the desired speed""" 
                            # stop shaking 
                            liconic.shaker_controller.stop_shaker()
                            # set shaking speed to new value 
                            liconic.shaker_controller.shaker_speed = new_shaker_speed
                            # restart shaking at new speed 
                            liconic.shaker_controller.activate_shaker()
                    else:
                        """not already shaking"""
                        # set shaking speed to new value (regardless of if already set to new value)
                        liconic.shaker_controller.shaker_speed = new_shaker_speed
                        # start shaking
                        liconic.shaker_controller.activate_shaker()
                    step_response.action_response = StepStatus.SUCCEEDED
                    step_response.action_msg = "Liconic shaker activated, shaker speed: " + str(liconic.shaker_controller.shaker_speed)
                except ValueError as val_err:
                    error_msg = "Error: shaker_speed argument must be an int"
                    print(error_msg)
                    state = ModuleStatus.IDLE
                    step_response.action_response=StepStatus.FAILED
                    step_response.action_log=error_msg
            elif action_handle == "end_shake":
                liconic.shaker_controller.stop_shaker()
                step_response.action_response = StepStatus.SUCCEEDED
                step_response.action_msg = "Liconic shaker stopped"
            elif action_handle == "load_plate":
                try:
                    stacker = action_args.get('stacker', None)
                    slot = action_args.get('slot', None)
                    plate_id = action_args.get('plate_id', None)
                    if stacker is None or slot is None:
                        stacker, slot = module_resources.get_next_free_slot_int()
                    if module_resources.is_location_occupied(stacker, slot):
                        step_response.action_response = StepStatus.FAILED
                        step_response.action_log = "load_plate command cannot be completed, already plate in given position"
                    else:
                        liconic.plate_handler.move_plate_from_transfer_station_to_slot(stacker, slot)
                        time.sleep(20)
                        module_resources.add_plate(plate_id, stacker, slot)
                        step_response.action_response = StepStatus.SUCCEEDED
                        step_response.action_msg = "Plate loaded into liconic stack " + str(stacker) + ", slot " + str(slot)
                except ValueError as val_err:
                    step_response.action_response = StepStatus.FAILED
                    step_response.action_msg = "Error: stacker and slot variables must be integers; plate_id required"
            elif action_handle == "unload_plate":
                try:
                    stacker = action_args.get('stacker', None)
                    slot = action_args.get('slot', None)
                    plate_id = action_args.get('plate_id', None)
                    if stacker == None or slot == None:
                        # get location based on plate id
                        stacker, slot = module_resources.find_plate(plate_id)

                        stacker, slot = module_resources.convert_stack_and_slot_int(stacker, slot)
                    if module_resources.is_location_occupied(stacker, slot):
                        if plate_id is None:
                            plate_id = module_resources.get_plate_id(stacker, slot)
                        liconic.plate_handler.move_plate_from_slot_to_transfer_station(stacker, slot)
                        if liconic.plate_handler.transfer_station_occupied: 
                            print("Liconic transfer station occupied")
                        else: 
                            print("Liconic transfer station empty")
                        time.sleep(18)
                        module_resources.remove_plate(plate_id)
                        step_response.action_response = StepStatus.SUCCEEDED
                        step_response.action_msg = "Plate unloaded from liconic stack " + str(stacker) + ", slot " + str(slot)
                    else:
                        raise Exception("No plate in location, can't unload.")
                except ValueError as val_err:
                    step_response.action_response = StepStatus.FAILED
                    step_response.action_msg = "Error: stacker and slot variables must be integers; plate_id required"
                
            else:
                # Handle Unsupported actions
                step_response.action_response=StepStatus.FAILED
                step_response.action_log="Unsupported action"
        except Exception as e:
            print(str(e))
            state = ModuleStatus.ERROR
            step_response.action_response=StepStatus.FAILED
            step_response.action_log=str(e)
        else:
            state = ModuleStatus.IDLE
    return step_response

def create_resource_file():
    '''
    if resource file does not exist, creates a blank one
    '''
    resources = {}
    for stack in range(4):
        curr_stack = "Stack"+str(stack+1)
        print(curr_stack)
        slot_dict = {}
        for slot in range(22):
            curr_slot = "Slot"+str(slot+1)
            slot_dict[curr_slot] = {
                "occupied": False,
                "time_added": "0",
                "plate_id": "NONE"
            }

        resources[curr_stack] = slot_dict
    
    return resources


if __name__ == "__main__":
    import uvicorn

    args = parse_args()
    
    uvicorn.run(
        "ur_rest_node:app",
        host=args.host,
        port=int(args.port),
        reload=True,
    )
