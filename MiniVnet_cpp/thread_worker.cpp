#include"thread_worker.h"

vector<Thread_Worker> _thread_pool;

void init_thread_pool() {
	for (int i = 0; i < _thread_num; i++) {
		_thread_pool.push_back(Thread_Worker());
	}
}

void routing_in_thread(Thread_Worker* thread_worker) {
	unique_lock<mutex> worker_lock(thread_worker->mtx);
	thread_worker->request_lock_worker.wait(worker_lock, thread_worker->is_main_ready);

	routing(*(thread_worker->route_group_ptr));
}