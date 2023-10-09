import sys
import urlib
is_py2 = sys.version[0] == '2'
if is_py2:
  from SimpleXMLRPCServer import SimpleXMLRPCServer
else:
  from xmlrpc.server import SimpleXMLRPCServer

def get_next_pose(p):
  assert type(p) is dict
  pose = urlib.poseToList(p)
  print("Received pose: " + str(pose))
  pose = [-0.18, -0.61, 0.23, 0, 3.12, 0.04];
  return urlib.listToPose(pose);

server = SimpleXMLRPCServer(("", 50000), allow_none=True)
server.RequestHandlerClass.protocol_version = "HTTP/1.1"
print("Listening on port 50000...")

server.register_function(get_next_pose, "get_next_pose")

server.serve_forever()
