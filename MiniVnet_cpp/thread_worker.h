#pragma once

#include <mutex>
#include <condition_variable>

#include"global.h"

class Thread_Worker {
public:

	// Variables to tell main thread that the job is done
	static mutex routing_done_mutex;
	static condition_variable routing_done_condition_variable;
	bool allow_main_continue = false;

	// Variables to tell worker to do the job
	const vector<reference_wrapper<Car>>* route_group_ptr = nullptr;
	map<string, string> routes_dict;
	static mutex request_worker_mutex;
	static condition_variable request_worker_condition_variable;
	static bool request_worker_ready;	// Initialize with "false" in thread_worker.cpp

	thread* worker_ptr;

	Thread_Worker() {
		worker_ptr = new thread(routing_in_thread, this);
	}
	~Thread_Worker() {
		delete worker_ptr;
	}
};
