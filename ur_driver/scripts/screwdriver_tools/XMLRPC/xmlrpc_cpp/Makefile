CFLAGS  = -g -Wall 
LDFLAGS = -lxmlrpc_server++ -lxmlrpc++ -lxmlrpc_server -lxmlrpc -lxmlrpc_server_pstream++ -lxmlrpc_packetsocket -lxmlrpc_xmlparse -lxmlrpc_xmltok -lxmlrpc_util -lxmlrpc_client++ -lxmlrpc_client -lcurl

TARGET = xmlrpc_camera 

all: $(TARGET)

clean:
	$(RM) *.o *~ $(TARGET)

SRCS = utilities.cpp server.cpp XMLRPCConverters.cpp

OBJS = $(SRCS:.cpp=.o)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) -c $<  -o $@
