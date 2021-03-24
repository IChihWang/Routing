#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

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

#define PORT 9996

using namespace std;


void ClearWinSock();
void initial_server_handler();