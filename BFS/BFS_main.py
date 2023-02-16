# importing parkages
import math
import json
import time
import numpy as np
import tracemalloc as trace
import pandas as pd
import matplotlib.pyplot as plt

from BFS import *

show_animation = True
pause_time = 0.01

def main():


    # Initiate time and memory calculation
    trace.start()
    start = time.time()
    print("Time calculation initiated")

    # defining obstacles and free spaces 
    ox, oy, fx, fy = [], [],[], [] 

    # read configuration settings from the json config file
    gmap_from_file = '../config15x20.json'
    with open(gmap_from_file) as config_env:
        param = json.load(config_env)

    # assign values for starting position, goal, size of grid, radius of the robot, etc, from the config settings just read
    sx = param['sx']  
    sy = param['sy']   
    gx = param['gx']   
    gy = param['gy']   
    grid_size = param['grid_size']  
    robot_radius = param['robot_radius']   
    map_xlsx = param['map_xlsx']
    fig_dim = param['fig_dim']

    # read environment from excel file using pandas
    gridmap = pd.read_excel(map_xlsx,header=None)
    data = gridmap.to_numpy()
    data
    # rearranging the array...
    data = data[::-1] 

    # determining obstacles or free space
    for iy in range(data.shape[0]):
        for ix in range(data.shape[1]):
            if data[iy, ix] == 1:
                ox.append(ix)
                oy.append(iy)
            else:
                fx.append(ix)
                fy.append(iy)


    if show_animation:  
            plt.figure(figsize=(fig_dim,fig_dim))
            plt.xlim(-1, data.shape[1])
            plt.ylim(-1, data.shape[0])
            plt.xticks(np.arange(0,data.shape[1],1))
            plt.yticks(np.arange(0,data.shape[0],1))
            plt.grid(True)
            plt.scatter(ox, oy, s=300, c='gray', marker='s')
            plt.scatter(fx, fy, s=300, c='cyan', marker='s') 

    # TO get path from start to goal, reverse rx,ry which are actually generated as a trace from goal to starting point
    def path(rx, ry):
        rx.reverse()  
        ry.reverse()

        # To form cordinates of the path, we convert to integers and zip the two lists together
        new_rx = map(int, rx)
        new_ry = map(int, ry) 
        path = [list(a) for a in zip(new_rx, new_ry)]

        # To print out the path to goal
        print("Path from start position to goal is: ", path)

        # To calculate the time it takes to reach the goal
        end = time.time()
        # print("Now to calculater processing time...")
        print("Total Time is: ", end-start)
        
        # To calculate memory used
        currentMem, highestMem = trace.get_traced_memory()
        print(f"Memory used is {currentMem / 10**6}MB; Highest was {highestMem / 10**6}MB")
        trace.stop()
        
        if show_animation:  
            for i in range(len(rx)-1):
                px = (rx[i], rx[i+1])
                py = (ry[i], ry[i+1])
                plt.plot(px, py, "-k")
                plt.pause(0.1)
            plt.show() 




    bfs = BFSPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = bfs.planning(sx, sy, gx, gy)
    path(rx, ry)



    

main()