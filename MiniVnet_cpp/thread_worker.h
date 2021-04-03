#pragma once

#include <mutex>
#include <condition_variable>

#include"global.h"

void foo()
{
	// Do something
}
class Thread_Worker {
public:
	vector<reference_wrapper<Car>>* route_group_ptr = nullptr;
	condition_variable request_lock_worker;
	condition_variable request_lock_main;
	thread* worker_ptr = nullptr;
	mutex mtx;

	bool is_main_ready = false;
	bool is_worker_ready = false;

	bool is_main_ready() { return is_main_ready != 0; }
	bool is_worker_ready() { return is_worker_ready != 0; }

	Thread_Worker() {
		worker_ptr = new thread(routing_in_thread, this);
	}
	~Thread_Worker() {
		delete worker_ptr;
	}
};
