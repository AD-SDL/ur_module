#pragma once

#include <cassert>
#include <map>
#include <xmlrpc-c/base.hpp>
#include <iostream>
#include <stdexcept>

typedef struct _Pose {
  double value[6];
} Pose;

extern const char *pose_struct_names[];

std::ostream &operator<<(std::ostream &os, Pose const &p);

Pose rpcvalueToPose(xmlrpc_c::cstruct const cpose);

xmlrpc_c::value_struct poseToRPCValue(Pose pose);

xmlrpc_c::value_nil valueToRPCValue(void);
xmlrpc_c::value_int valueToRPCValue(int v);
xmlrpc_c::value_double valueToRPCValue(double v);
xmlrpc_c::value_boolean valueToRPCValue(bool v);
xmlrpc_c::value_string valueToRPCValue(std::string v);

/**
 * @brief Creates a vector<double> from the UR script list type.
 * Note, the UR Controller can have mixed float/integers in it's lists.
 */
std::vector<double> rpcvalueToVector(xmlrpc_c::carray arr);

template <typename T>
xmlrpc_c::value_array vectorToRPCValue(std::vector<T> vec){
  xmlrpc_c::carray arr(vec.size());
  for(unsigned int i = 0; i < vec.size(); ++i){
    arr[i] = valueToRPCValue(vec[i]);
  }
  return arr;
}
