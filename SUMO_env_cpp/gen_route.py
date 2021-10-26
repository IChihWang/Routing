from __future__ import absolute_import
from __future__ import print_function

import os
import sys

import optparse
import random
import numpy

import traci

import config as cfg
import json



def generate_routefile_with_src_dst(inter_size, arrival_rate, rand_seed, time_steps):

    path = "data/routes/"
    file_name = str(inter_size) + "_" + str(arrival_rate) + "_" + str(rand_seed)
    routes = open(path + file_name + ".rou.xml", "w")
    src_dst = open(path + file_name + "_src_dst" + ".json", "w")

    print("<routes>\n", file=routes)

    for i in range(5,10):
        vType_str = '\t<vType id="car%i" accel="200.0" decel="200.0" speedFactor="2" sigma="0.0" length="%i" minGap="%f" maxSpeed="%f" tau="%f" carFollowModel="CACC" color="255,255,255"/>' % (i, i, cfg.HEADWAY, cfg.MAX_SPEED, cfg.TIME_STEP);

        print(vType_str, file=routes)

    route_list = []

    car_src_dst_dict = dict()   # {car_id: (src, dst)}
    dst_idx_dst_id_map = dict()

    route_str = "\n"
    for x_idx in range(1, inter_size+1):

        y_idx = 1
        route_id = (4*inter_size-1) - (0*inter_size + (x_idx-1))
        src_lane = "%03d"%(y_idx) + "_" + "%03d"%(x_idx) + "_1"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        dst_id = "%03d"%(y_idx-1) + "_" + "%03d"%(x_idx)
        dst_idx_dst_id_map[route_id] = dst_id


        y_idx = inter_size
        route_id = (4*inter_size-1) - (1*inter_size + (x_idx-1))

        src_lane = "%03d"%(x_idx) + "_" + "%03d"%(y_idx) + "_2"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        dst_id = "%03d"%(x_idx) + "_" + "%03d"%(y_idx+1)
        dst_idx_dst_id_map[route_id] = dst_id


        y_idx = inter_size
        route_id = (4*inter_size-1) - (2*inter_size + (inter_size - x_idx))
        src_lane = "%03d"%(y_idx) + "_" + "%03d"%(x_idx) + "_3"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        dst_id = "%03d"%(y_idx+1) + "_" + "%03d"%(x_idx)
        dst_idx_dst_id_map[route_id] = dst_id

        y_idx = 1
        route_id = (4*inter_size-1) - (3*inter_size + (inter_size - x_idx))
        src_lane = "%03d"%(x_idx) + "_" + "%03d"%(y_idx) + "_4"
        route_str += "\t<route id=\""
        route_str += str(route_id)
        route_str += "\" edges=\""
        route_str += " " + str(src_lane)
        route_str += "\"/>\n"
        route_list.append(str(route_id))
        dst_id = "%03d"%(x_idx) + "_" + "%03d"%(y_idx-1)
        dst_idx_dst_id_map[route_id] = dst_id

    print(route_str, file=routes)


    #dir_prob = numpy.random.uniform(0, arrival_rate*1.5, len(route_list)).tolist()
    #print(dir_prob)
    #dir_prob = [arrival_rate]*len(route_list)

    vehNr = 0
    for i in range(time_steps):
        for idx_route in range(len(route_list)):
            route = route_list[idx_route]
            #if random.uniform(0, 1) < dir_prob[idx_route]:
            if random.uniform(0, 1) < random.uniform(0, arrival_rate):
                car_length = random.randrange(5,10)
                veh_str = "\t<vehicle id=\"car"
                lane_r = random.randrange(cfg.LANE_NUM_PER_DIRECTION)

                veh_str += '_%i" type="car%i" route="%s" depart="%i" departLane = "%i" departSpeed="%f"/>' % (vehNr, car_length, route, i, lane_r, cfg.MAX_SPEED);


                src_node_idx = int(route)
                # Genterate destination
                dst_node_idx = src_node_idx
                #while (src_node_idx-dst_node_idx)%(inter_size*4) < inter_size-1 or (dst_node_idx-src_node_idx)%(inter_size*4) < inter_size-1:
                while dst_node_idx == src_node_idx:
                    dst_node_idx = random.randrange(0,inter_size*4)

                #while src_node_idx == dst_node_idx:
                #    dst_node_idx = random.randrange(0,cfg.INTER_SIZE*4)

                car_src_dst_dict["car_"+str(vehNr)] = (src_node_idx, dst_node_idx, dst_idx_dst_id_map[dst_node_idx])

                print(veh_str, file=routes)

                vehNr += 1


    print("</routes>", file=routes)

    json.dump(car_src_dst_dict, src_dst)

    return vehNr
