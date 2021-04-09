#include "server.h"

using namespace std;

uint8_t _grid_size;
float _schedule_period;
uint8_t _routing_period_num;
float _GZ_BZ_CCZ_len;
uint8_t _HEADWAY;
float _V_MAX;
float _TURN_SPEED;
uint16_t _TOTAL_LEN;
float _routing_period;

SOCKET initial_server_handler() {
#if defined WIN32
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("error at WSAStartup\n");
		exit(EXIT_FAILURE);
	}
#endif

	SOCKET server_fd;
	server_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (server_fd <= 0) {
		cout << "socket creation failed." << endl;
		closesocket(server_fd);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	char opt = 1;
	// Forcefully attaching socket to the port 8080
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
		cout << "socket setsockopt failed." << endl;
		closesocket(server_fd);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	// Server address construction
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = INADDR_ANY;
	sockAddr.sin_port = htons(PORT);

	// Forcefully attaching socket to the port 8080
	if (bind(server_fd, (struct sockaddr*)&sockAddr, sizeof(sockAddr)) < 0) {
		cout << "socket bind failed." << endl;
		closesocket(server_fd);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	if (listen(server_fd, 1) < 0) {
		cout << "socket listen failed." << endl;
		closesocket(server_fd);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	// Accepting connections
	SOCKET new_socket;
	int addrlen = sizeof(sockAddr);

	if ((new_socket = accept(server_fd, (struct sockaddr*)&sockAddr, (socklen_t*)&addrlen)) < 0)
	{
		cout << "socket accepting failed." << endl;
		closesocket(server_fd);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	char buffer[1024] = { 0 };
	recv(new_socket, buffer, 1024, 0);

	cout << buffer << endl;
	stringstream ss(buffer);
	vector<float> results;
	for (int i = 0; i < 8; i++) {
		string substr;
		getline(ss, substr, ':');
		getline(ss, substr, ':');
		stringstream tmp_ss(substr);
		float tem_float;
		tmp_ss >> tem_float;
		results.push_back(tem_float);
	}

	_grid_size = uint8_t(results[0]);
	_schedule_period = results[1];
	_routing_period_num = uint8_t(results[2]);
	_GZ_BZ_CCZ_len = results[3];
	_HEADWAY = uint8_t(results[4]);
	_V_MAX = results[5];
	_TURN_SPEED = results[6];
	_TOTAL_LEN = uint16_t(results[7]);
	_routing_period = _schedule_period * _routing_period_num;

	cout << _grid_size << " " << _schedule_period << " "
		<< _routing_period_num << " " << _GZ_BZ_CCZ_len << " "
		<< _HEADWAY << " " << _V_MAX << " "
		<< _TURN_SPEED << " " << _TOTAL_LEN << endl;

	char hello[] = "Got it ;@";
	send(new_socket, hello, (int)strlen(hello), 0);

	return new_socket;
}

void ClearWinSock() {
#if defined WIN32
	WSACleanup();
#endif
}
