# importing parkages
import math
import json
import time
import numpy as np
import tracemalloc
import pandas as pd
import matplotlib.pyplot as plt


show_animation = True

pause_time = 0.1


class BFSPlanning:
    def __init__(self, ox, oy, resolution, robotRadius):
        
        #Here we define the  grid map and motion model
        """
       when called, function will require-
       ox, oy: x and y cordinates for obstacles
       rosolution: Map's resolution
       robotRad: Radius of the robot
       """

        self.resolution = resolution
        self.robotRadius = robotRadius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model_4n()
  
    
    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            self.parent = parent

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index) + "," + str(self.parent)
            
    def planning(self, sx, sy, gx, gy):
        """
        Path planning for Breadth First search 

        takes the following parameter:
            sx: start position for x
            sy: start position for y
            gx: goal position for x
            gy: goal position for y

        it outputs:
            rx: a list of x positions that make up final path from goal to sx
            ry: a list of y positions that make up final path from goal to sy
        """
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                           self.calc_xy_index(sy, self.min_y), 0.0, -1, None)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                          self.calc_xy_index(gy, self.min_y), 0.0, -1, None)
        
        ##plot start node and goal node on map specifying their cell colour
        plt.scatter(start_node.x, start_node.y, s=400, c='green', marker='s')
        plt.scatter(goal_node.x, goal_node.y, s=400, c='orange', marker='s')  

        #Create 2 dictionaries, open and closed set add start node to open set

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node


        while 1:
            if len(open_set) == 0:
                print("No more node to visit")
                break
            
            # remove current from open_set and get its index . Note, the list uses FIFO
            current = open_set.pop(list(open_set.keys())[0])

            c_id = self.calc_grid_index(current)

            ##add explored node to closed_set (i.e the list of already visited nodes)
            closed_set[c_id] = current

            # show graph
            if show_animation: 

                plt.scatter(self.calc_grid_position(current.x, self.min_x),
                            self.calc_grid_position(current.y, self.min_y),
                             s=300, c='yellow', marker='s')
                plt.pause(pause_time)       

                # To stop simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape'
                                              else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)
                
                
            if len(closed_set) >= 2:
                locx, locy = self.calc_final_path(current, closed_set)
                
                for i in range(len(locx)-1):
                    px = (locx[i], locx[i+1])
                    py = (locy[i], locy[i+1])
                    plt.plot(px, py, "-m", linewidth=4)
                    plt.pause(pause_time) 
            # Goal check
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Eureka, Goal Found!")

                plt.scatter(current.x,
                            current.y,
                            s=300, c='orange', marker='s') 
                plt.pause(pause_time)

                
                if len(closed_set) >= 2:
                    locx, locy = self.calc_final_path(current, closed_set)                
                    for i in range(len(locx)-1):
                        px = (locx[i], locx[i+1])
                        py = (locy[i], locy[i+1])
                        plt.plot(px, py, "-m", linewidth=4)
                        plt.pause(pause_time)

                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                
                print("Total cost = ", goal_node.cost )
                
                break


            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id, None)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    plt.scatter(node.x,
                                node.y,
                                s=100, c='black', marker='s')
                    plt.pause(pause_time)                    
                    continue

                if (n_id not in closed_set) and (n_id not in open_set):
                    node.parent = current
                    open_set[n_id] = node
                    plt.scatter(node.x,
                                node.y,
                                s=100, c='blue', marker='s')
                    plt.pause(pause_time)              
                             
                           
        #graph
        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

            
    def calc_final_path(self, goal_node, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        print(goal_node.parent_index)
        n = closedset[goal_node.parent_index]
        # print(n)
        
        ##get index from goal to start
        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            n = n.parent

        return rx, ry

    ##get coordinates from index in grid system
    def calc_grid_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    ##get index from coordinates based on x or y
    ## use in determining start node
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    ##get index from coordinates based on x and y
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.xwidth + (node.x - self.min_x)

    ##verify node if it safe or not when exploring 
    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[node.x][node.y]:
            return False

        return True

    ##generate obstacles
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

        self.xwidth = round((self.maxx - self.min_x) / self.resolution)
        self.ywidth = round((self.maxy - self.min_y) / self.resolution)


        # obstacle map generation
        self.obmap = [[False for _ in range(self.ywidth)]
                      for _ in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robotRadius:
                        self.obmap[ix][iy] = True
                        break

    
    ## The motion model
    @staticmethod
    def get_motion_model_4n():
        """
        Get all possible 4-connectivity movements.
        :returns a list of movements with cost [[dx, dy, cost]]
        """
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1]]
        return motion