from __future__ import absolute_import
from __future__ import print_function
import os
import sys
k = 0

length = 227.2  # Length inside the network: actual length = length - 27.2
attach = 286.4  # Additional length around the network: actual length = attach + 213.6
def gen_node(x, y):
    with open("./tmp/lane.nod.xml", "w") as nets:
        print('<nodes>', file=nets)
        for i in range(x + 2): # y-axis
            for j in range(y + 2): # x-axis 1
                if i == 0 or i == x+1:
                    if j != 0 and j != y+1:
                        if i == 0:
                            print('<node id="%03d_%03d" x="%f" y="%f" type="priority" />' %(i, j , i*length-attach,j*length),file=nets)
                        if i == x+1:
                            print('<node id="%03d_%03d" x="%f" y="%f" type="priority" />' %(i, j , i*length+attach,j*length),file=nets)
                elif j == 0:
                    print('<node id="%03d_%03d" x="%f" y="%f" type="priority" />' %(i, j , i*length,j*length-attach),file=nets)
                elif j == y+1:
                    print('<node id="%03d_%03d" x="%f" y="%f" type="priority" />' %(i, j , i*length,j*length+attach),file=nets)
                else:
                    print('<node id="%03d_%03d" x="%f" y="%f" type="priority" />' %(i, j , i*length,j*length),file=nets)


        print('</nodes>', file=nets)


def gen_edge(x, y):
    with open("./tmp/lane.edg.xml", "w") as edges:
        print('<edges>', file=edges)

        for j in range(1, y + 1):
            for i in range(1, x+1):
                # Left
                print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(i, j, '_1', i-1, j, i, j),file=edges)
                # Up
                print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(i, j, '_2', i, j+1, i, j),file=edges)
                # Right
                print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(i, j, '_3', i+1, j, i, j),file=edges)
                # Down
                print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(i, j, '_4', i, j-1, i, j),file=edges)

        # x-Axis going out
        for i in range(1, x+1):
            print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(i, 0, '_2', i, 1, i, 0),file=edges)
            print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(i, y+1, '_4', i, y, i, y+1),file=edges)
        for j in range(1, y+1):
            print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(0, j, '_3', 1, j, 0, j),file=edges)
            print('<edge id="%03d_%03d%s" from="%03d_%03d" to="%03d_%03d" priority="3" numLanes="3" speed= "11.18" width="3.20"/>' %(x+1, j, '_1', x, j, x+1, j),file=edges)

        print('</edges>', file=edges)

def gen_connection(x, y):
    with open("./tmp/lane.con.xml", "w") as cons:
        print('<connections>', file=cons)

        for j in range(1, y + 1):
            for i in range(1, x+1):
                for k in range(3):
                # 1-left
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_1', i, j+1, '_4',k,k),file=cons)
                # 1-straight
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_1', i+1, j, '_1',k,k),file=cons)
                # 1-right
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_1', i, j-1, '_2',k,k),file=cons)

                # 2-left
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_2', i-1, j, '_3',k,k),file=cons)
                # 2-straight
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_2', i, j-1, '_2',k,k),file=cons)
                # 2-right
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_2', i+1, j, '_1',k,k),file=cons)

                # 3-left
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_3', i, j-1, '_2',k,k),file=cons)
                # 3-straight
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_3', i-1, j, '_3',k,k),file=cons)
                # 3-right
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_3', i, j+1, '_4',k,k),file=cons)

                # 4-left
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_4', i-1, j, '_3',k,k),file=cons)
                # 4-straight
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_4', i, j+1, '_4',k,k),file=cons)
                # 4-right
                    print('<connection from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, j, '_4', i+1, j, '_1',k,k),file=cons)
        for i in range(1, x+1):
            print('<delete from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, 0, '_2', i, 1, '_4',2,2),file=cons)
            print('<delete from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(i, x+1, '_4', i, x, '_2',2,2),file=cons)
        for j in range(1, y+1):
            print('<delete from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(0, j, '_3', 1, j, '_1',2,2),file=cons)
            print('<delete from="%03d_%03d%s" to="%03d_%03d%s" fromLane="%d" toLane="%d"/>' %(x+1, j, '_1', x, j, '_3',2,2),file=cons)

        print('</connections>', file=cons)

def gen_tlc(x, y):
    with open("./tmp/lane.tlc.xml", "w") as tlcs:
        for i in range(x + 2): # y-axis
            for j in range(y + 2): # x-axis 1
                if i == 0 or i == x+1:
                    if j != 0 and j != y+1:
                        print('<tlLogic id="%03d_%03d" type="static">' %(i, j), file=tlcs)
                        print('<phase duration="31" state="GGggggggGGgggggg"/>' ,file=tlcs)
                        print('</tlLogic>', file=tlcs)
os.system('cd /usr/local/Cellar/sumo/1.4.0/share/sumo/tools')
def gen_net(i):
    gen_node(i, i)
    gen_edge(i, i)
    gen_connection(i, i)
for i in range(2,11):
    gen_net(i)
    node = './tmp/lane.nod.xml'
    edge = './tmp/lane.edg.xml'
    con = './tmp/lane.con.xml'
    out = './tmp/lane{}by{}.net.xml'.format(i, i)

    os.system("netconvert --node-files={} --edge-files={} --connection-files={} --output-file={}".format(node,edge,con,out))
