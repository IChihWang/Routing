#include"thread_worker.h"

vector<Thread_Worker*> _thread_pool;

mutex Thread_Worker::routing_done_mutex;
condition_variable Thread_Worker::routing_done_condition_variable;

mutex Thread_Worker::request_worker_mutex;
condition_variable Thread_Worker::request_worker_condition_variable;

void init_thread_pool() {
	for (uint8_t i = 0; i < _thread_num; i++) {
		_thread_pool.push_back(new Thread_Worker());
	}
}

void terminate_thread_pool() {
	for (uint8_t i = 0; i < _thread_num; i++) {
		delete _thread_pool[i];
	}
}


void routing_in_thread(Thread_Worker* thread_worker) {
	while (true) {
		// Important: thread_workers have to be clean!!!!!!! (e.g., routes_dict is empty)
		// Wait for the command from main

		{
			unique_lock<mutex> worker_lock(thread_worker->request_worker_mutex);
			thread_worker->request_worker_condition_variable.wait(worker_lock, [thread_worker] {return thread_worker->request_worker_ready; });
			thread_worker->request_worker_ready = false;	// to prevent superious wakeup
		}

		// Start the routing
		map<string, vector<Node_in_Path>> result = routing(*(thread_worker->route_group_ptr));
		// Get the result
		for (const auto& [car_id, path_data] : result) {
			string turning_str = "";
			for (Node_in_Path node_in_path_data : path_data) {
				turning_str += node_in_path_data.turning;
			}
			_car_dict[car_id].path_data = path_data;

			thread_worker->routes_dict[car_id] = turning_str;
		}
		// Done task, notify main thread
		thread_worker->allow_main_continue = true;
		thread_worker->routing_done_condition_variable.notify_all();
	}
}

// Return true if all workers done their jobs (for main to stop waiting)
bool check_all_thread_done() {
	bool is_all_thread_done = true;
	for (const auto& worker_ptr : _thread_pool) {
		is_all_thread_done &= worker_ptr->allow_main_continue;
	}
	return is_all_thread_done;
}

map<string, string>& routing_with_groups_thread(const vector<vector<reference_wrapper<Car>>>& route_groups, map<string, string>& routes_dict) {
	// Dispatch workload
	for (uint8_t thread_i = 0; thread_i < _thread_num; thread_i++) {
		const vector<reference_wrapper<Car>>& route_group = route_groups[thread_i];
		(*(_thread_pool[thread_i])).route_group_ptr = &route_group;
		(*(_thread_pool[thread_i])).request_worker_ready = true;
	}

	// Start workers
	{
		unique_lock<mutex> worker_lock(Thread_Worker::request_worker_mutex);
		Thread_Worker::request_worker_condition_variable.notify_all();
	}

	// Wait for results
	{
		unique_lock<mutex> main_thread_lock(Thread_Worker::routing_done_mutex);
		Thread_Worker::routing_done_condition_variable.wait(main_thread_lock, check_all_thread_done);
	}

	for (uint8_t thread_i = 0; thread_i < _thread_num; thread_i++) {
		(*(_thread_pool[thread_i])).route_group_ptr = nullptr;
		(*(_thread_pool[thread_i])).allow_main_continue = false;
		(*(_thread_pool[thread_i])).routes_dict.clear();
	}

	// Get results (copy the results from threads to routes_dict)
	for (const auto& worker_ptr: _thread_pool) {
		for (const auto& [car_id, turning_str] : worker_ptr->routes_dict) {
			routes_dict[car_id] = turning_str;
		}
	}

	return routes_dict;
}