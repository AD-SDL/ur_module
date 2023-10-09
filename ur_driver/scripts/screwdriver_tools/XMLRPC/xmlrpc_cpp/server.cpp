#include <unistd.h>
#include <cassert>
#include <iostream>
#include <signal.h>
#include <cstdlib>

using namespace std;

#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_pstream.hpp>

#include "XMLRPCConverters.hpp"
#include "utilities.hpp"

xmlrpc_c::registry theRegistry;

void register_helper(std::string name, xmlrpc_c::method * const ptr){
  xmlrpc_c::methodPtr const method(ptr);
  theRegistry.addMethod(name, method);
}

class PoseAdapter : public xmlrpc_c::method {
public:
  PoseAdapter(){
    this->_signature = "S:S";
  	}
  
  void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value *   const  retvalP){
     Pose pose = rpcvalueToPose(paramList.getStruct(0));

     paramList.verifyEnd(1);

     cout << "Received pose: " << pose << endl;
     pose.value[0] = -0.18;
     pose.value[1] = -0.61;
     pose.value[2] = 0.23;
     pose.value[3] = 0;
     pose.value[4] = 3.12;
     pose.value[5] = 0.04;

     *retvalP = poseToRPCValue(pose);
   }
};

int main(int const argc, const char ** const argv) {

  int ssock = createSocket(50000);

  cout << "Listening on port 50000..." << endl;

  register_helper("get_next_pose", new PoseAdapter);

  while(true){

    int sockfd = waitForConnection(ssock);

    try {

          xmlrpc_c::serverPstream server(
              xmlrpc_c::serverPstream::constrOpt()
              .socketFd(sockfd)
              .registryP(&theRegistry));

          server.runSerial();

      } catch (exception const& e) {
          cerr << "Something threw an error: " << e.what() << endl;
      }
  }
  return 0;
}
