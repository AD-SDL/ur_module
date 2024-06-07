"""REST-based node for UR robots"""

from wei.modules.rest_module import RESTModule

rest_module = RESTModule(
    name="pf400_node",
    version="0.0.1",
    description="A node to control the pf400 plate moving robot",
    model="pf400",
)
rest_module.arg_parser.add_argument(
    "--pf400_ip", type=str, help="pf400 ip value", default="146.137.240.35"
)
rest_module.arg_parser.add_argument(
    "--pf400_port", type=int, help="pf400 port value", default=10100
)


rest_module.start()
