
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>

#define WIN32
#if defined WIN32
#include <winsock.h>
# pragma comment(lib,"ws2_32.lib") //Winsock Library
typedef int socklen_t;
#else

#define closesocket close
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include "global.h"

#define SUMO_PORT 9997
#define ROUTER_PORT 9996
//#define IP_ADDR "127.0.0.1"
#define IP_ADDR "140.113.236.57"

extern SOCKET client_sock;

// Functions in server.cpp
void ClearWinSock();
SOCKET initial_client_handler();


