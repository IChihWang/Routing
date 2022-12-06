#pragma once

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include <cstring>

typedef int SOCKET;

//#define WIN32
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

#define PORT 29996

// Functions in server.cpp
void ClearWinSock();
SOCKET initial_server_handler();
