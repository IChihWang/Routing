#include "thread_worker.h"
#include "server.h"

void call_routing_thread(Thread_Worker* thread_worker) {
	while (true) {
		// Wait for the command from main
		{
			unique_lock<mutex> worker_lock(thread_worker->request_worker_mutex);
			thread_worker->request_worker_condition_variable.wait(worker_lock, [thread_worker] {return thread_worker->request_worker_ready; });
			thread_worker->request_worker_ready = false;	// to prevent superious wakeup
		}

		// Get the request from main thread
		string send_str = thread_worker->route_request;
		thread_worker->route_request = "";
		if (send_str == "End Connection") {
			break;
		}
		send_str += '@';

		// Send request
		send(client_sock, send_str.c_str(), int(send_str.length()), 0);

		// Get route results
		string in_str = "";
		while (in_str.length() == 0 || in_str.back() != '@') {
			char buffer[1024] = { 0 };
			int n_recv = recv(client_sock, buffer, 1023, 0);
			if (n_recv <= 0) {
				return;
			}
			in_str += buffer;
		}
		in_str.pop_back();

		thread_worker->route_result = in_str;

		// Done task, notify main thread
		thread_worker->allow_main_continue = true;
		thread_worker->routing_done_condition_variable.notify_all();
	}
}