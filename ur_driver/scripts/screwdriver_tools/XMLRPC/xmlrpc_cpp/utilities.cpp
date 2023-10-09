#include "utilities.hpp"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <iostream>

using namespace std;

void error(const char* format, ...){
  static char buffer[255];
  va_list var_args;
  va_start(var_args, format);  //The variable argument list starts after the format argument
  ::vsnprintf(buffer, 255, format, var_args);
  ::printf("ERROR: %s\n", buffer);
  va_end(var_args);
  exit(1);
}

int createSocket(int port){

  int sockfd;
  struct sockaddr_in serv_addr;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
     error("ERROR opening socket");
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
  if (bind(sockfd, (struct sockaddr *) &serv_addr,
           sizeof(serv_addr)) < 0)
           error("ERROR on binding");

  return sockfd;
}

int waitForConnection(int sockfd){

  if(listen(sockfd,5)){
    error("Waiting for connection failed.");
  }

  int on = 1;
  int setsockoptSuccess = setsockopt (sockfd, SOL_TCP, TCP_NODELAY, &on, sizeof (on));
  if (setsockoptSuccess != 0) {
    error("setsockopt ERROR = %i", setsockoptSuccess);
  }

  return sockfd;
}
