#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <vector>

template <typename T>
std::ostream &operator<<(std::ostream &os, std::vector<T> const &v){
  os << "[";
  if(v.size() > 0)
    os << v[0];
  for(unsigned int i = 1; i < v.size(); ++i){
    os << ", " << v[i];
  }
  return os << "]";
}

void error(const char* format, ...);

int createSocket(int port);
int waitForConnection(int sockfd);
