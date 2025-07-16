
from globus_compute_sdk import Client, Executor
gcc = Client()

registered_id = '188601cd-e9fa-49ab-adb4-fdf0439351d6'
func_uuid = 'a01e26f1-bb08-46b5-84be-c61405f9ecbc'
gce = Executor(endpoint_id = registered_id)

task_id = gcc.run(endpoint_id=registered_id, function_id = func_uuid)

future = gce.submit_to_registered_function(function_id = func_uuid)



future2 = gce.submit_to_registered_function(function_id="8339fe94-c34b-4c77-ae78-8313c2bef0f9")
print(future.result())
print(future2.result())
print(task_id)