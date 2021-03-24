#include "main.h"

void initial_server_handler() {
	cout << aaa << endl;
#if defined WIN32
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("error at WSAStartup\n");
		exit(EXIT_FAILURE);
	}
#endif

	int server_fd;
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
	int new_socket;
	int addrlen = sizeof(sockAddr);

	if ((new_socket = accept(server_fd, (struct sockaddr*)&sockAddr, (socklen_t*)&addrlen)) < 0)
	{
		cout << "socket accepting failed." << endl;
		closesocket(server_fd);
		ClearWinSock();
		exit(EXIT_FAILURE);
	}

	int valread;
	char buffer[1024] = { 0 };
	valread = recv(new_socket, buffer, 1024, 0);
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

	int grid_size = results[0];
	float schedule_period = results[1];
	int routing_period_num = results[2];
	float GZ_BZ_CCZ_len = results[3];
	float HEADWAY = results[4];
	float V_MAX = results[5];
	float TURN_SPEED = results[6];
	float TOTAL_LEN = results[7];
	cout << grid_size << " " << schedule_period << " "
		<< routing_period_num << " " << GZ_BZ_CCZ_len << " "
		<< HEADWAY << " " << V_MAX << " "
		<< TURN_SPEED << " " << TOTAL_LEN << endl;

	char hello[] = "Hello from server";
	send(new_socket, hello, strlen(hello), 0);
	printf("Hello message sent\n");
}

void ClearWinSock() {
#if defined WIN32
	WSACleanup();
#endif
}