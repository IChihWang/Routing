#include <string>
#include <sstream>
#include "server.h"

using namespace std;

uint8_t _grid_size;
SOCKET client_sock;


string get_string_from_double(double num, uint8_t percision) {
	stringstream ss;
	ss << std::fixed << std::setprecision(percision) << num;
	return ss.str();
}

SOCKET initial_client_handler() {
#if defined WIN32
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("error at WSAStartup\n");
		exit(EXIT_FAILURE);
	}
#endif

	client_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (client_sock <= 0) {
		cout << "socket creation failed." << endl;
		closesocket(client_sock);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	// Server address construction
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = inet_addr(IP_ADDR);
	sockAddr.sin_port = htons(ROUTER_PORT);

	// Connect socket to the port
	if (connect(client_sock, (SOCKADDR*)&sockAddr, sizeof(sockAddr)) < 0) {
		cout << "socket connect failed." << endl;
		closesocket(client_sock);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	string init_info = "";
	init_info += "My_grid_size:" + to_string(_grid_size);
	init_info += ":My_schedule_period:" + get_string_from_double(SCHEDULING_PERIOD, 2);
	init_info += ":My_routing_period_num:" + to_string(ROUTING_PERIOD_NUM);
	init_info += ":GZ_BZ_CCZ_len:" + get_string_from_double(GZ_LEN + BZ_LEN + CCZ_LEN, 1);
	init_info += ":HEADWAY:" + to_string(HEADWAY);
	init_info += ":V_MAX:" + get_string_from_double(V_MAX, 2);
	init_info += ":TURN_SPEED:" + get_string_from_double(TURN_SPEED, 2);
	init_info += ":TOTAL_LEN:" + get_string_from_double(TOTAL_LEN, 1);
	init_info += ":CHOOSE_CAR_OPTION:" + to_string(_CHOOSE_CAR_OPTION);
	init_info += ":TOP_N_CONGESTED:" + to_string(_TOP_N_CONGESTED);
	init_info += ":_THREAD_NUM:" + to_string(_THREAD_NUM);

	send(client_sock, init_info.c_str(), int(init_info.length()), 0);

	char buffer[1024] = { 0 };
	recv(client_sock, buffer, 1024, 0);

	cout << "Server replies: " << buffer << endl;

	return client_sock;
}

void ClearWinSock() {
#if defined WIN32
	WSACleanup();
#endif
}
