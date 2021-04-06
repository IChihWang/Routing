#include <iostream>
#include <sstream>
#include <string>
#include "global.h"
#include "server.h"

using namespace std;


SUMO_Traci traci;
uint16_t _N_TIME_STEP;
float _TIME_STEP;

void initial_sumo();

int main(int argc, char* argv[])
{
    cout << "Usage: ./main <grid_size>" << endl;
    // Parse the input
    if (argc < 2) {
        string grid_size_str = argv[1];
        _grid_size = stoi(grid_size_str);
    }

    // Initialization
    initial_client_handler();
    initial_sumo();

    Thread_Worker router_thread;     // New thread to send/receive routing requests/results  (do this after initial_client_handler();)

    // TODO: read src_dst_dict


    // Run sumo simulation (run_sumo in python version)
    traci.simulationStep(5);

    // TODO: SUMO simulation
    // TODO: get result from router first before updating the str

    traci.close();

    return 0;
}

// Initial SUMO
void initial_sumo() {
    traci.connect("localhost", SUMO_PORT);
}