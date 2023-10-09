#include "XMLRPCConverters.hpp"

#include <cassert>
#include <map>

const char *pose_struct_names[6] = {"x", "y", "z", "rx", "ry", "rz"};

std::ostream &operator<<(std::ostream &os, Pose const &p) {
  return os << "p[" << p.value[0] << ", " << p.value[1] << ", " << p.value[2] << ", " << p.value[3] << ", " << p.value[4] << ", " << p.value[5] << "]";
}

Pose rpcvalueToPose(xmlrpc_c::cstruct const cpose){
  assert(cpose.size() == 6);

  Pose pose;
  for(unsigned int i = 0; i < 6; ++i){
    xmlrpc_c::value v = cpose.at(pose_struct_names[i]);
    pose.value[i] = static_cast<xmlrpc_c::value_double>(v);
  }

  return pose;
}

using namespace std;

xmlrpc_c::value_struct poseToRPCValue(Pose pose){
  map<string, xmlrpc_c::value> structData;

  for(unsigned int i = 0; i < 6; ++i){
    pair<string, xmlrpc_c::value_double> member(pose_struct_names[i], pose.value[i]);
    structData.insert(member);
  }

  return structData;
}

xmlrpc_c::value_nil valueToRPCValue(void){
  return xmlrpc_c::value_nil();
}


xmlrpc_c::value_int valueToRPCValue(int v){
  return xmlrpc_c::value_int(v);
}

xmlrpc_c::value_double valueToRPCValue(double v){
  return xmlrpc_c::value_double(v);
}

xmlrpc_c::value_boolean valueToRPCValue(bool v){
  return xmlrpc_c::value_boolean(v);
}

xmlrpc_c::value_string valueToRPCValue(std::string v){
  return xmlrpc_c::value_string(v);
}

std::vector<double> rpcvalueToVector(xmlrpc_c::carray arr){
  std::vector<double> vec(arr.size(), 0);
  for(unsigned int i = 0; i < arr.size(); ++i){
    switch(arr[i].type()){
      case(xmlrpc_c::value::TYPE_INT):
        vec[i] = (double)static_cast<xmlrpc_c::value_int>(arr[i]).cvalue();
        break;
      case(xmlrpc_c::value::TYPE_DOUBLE):
        vec[i] = static_cast<xmlrpc_c::value_double>(arr[i]).cvalue();
        break;
      default:
        throw std::runtime_error("Type not recognized!");
        break;
    }
  }
  return vec;
}
