#pragma once

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>

using namespace std;

class Thread_Worker;
void call_routing_thread(Thread_Worker* thread_worker);

class Thread_Worker {
public:

	// Variables to tell main thread that the job is done
	mutex routing_done_mutex;
	condition_variable routing_done_condition_variable;
	bool allow_main_continue = false;
	string route_result = "";

	// Variables to tell worker to do the job
	string route_request = "";			// Clean up after the data is read
	mutex request_worker_mutex;
	condition_variable request_worker_condition_variable;
	bool request_worker_ready = false;

	thread* worker_ptr;

	Thread_Worker() {
		worker_ptr = new thread(call_routing_thread, this);
	}
	~Thread_Worker() {
		if (worker_ptr->joinable())
			delete worker_ptr;
	}
};
