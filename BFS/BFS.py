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
choice = int(input("select the operation you want to perform.\n1)BFS GRAPH\n2)BFS TREE\n"))

# Creating a class for Breadth First Search
class BFSPlanner:
    def __init__(self, ox, oy, resolution, robotRadius):
        
        """
        when called, function will require the following:
        ox, oy - the x and y cordinates for obstacles
        rosolution - Map's resolution for display
        robotRadius: Radius of the robot
        it returns a string of all the submited parameters
        """

        self.resolution = resolution
        self.robotRadius = robotRadius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model_8n()
  
    
    class Node:
        def __init__(self, x, y, cost, parent_index, parent):
            self.x = x  # index of the node on the grid-map
            self.y = y  # index of the node on the grid-map
            self.cost = cost # cost of getting to this node
            self.parent_index = parent_index # the index of the parent node
            self.parent = parent

        # return a string containing the parameters of the node
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
        
        #plot start node and goal node on map specifying their cell colour
        if show_animation:
            plt.scatter(start_node.x, start_node.y, s=400, c='green', marker='s')
            plt.scatter(goal_node.x, goal_node.y, s=400, c='orange', marker='s')  

        #Create 2 dictionaries, open and closed set, add start node to open set
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node


        while 1:
            #if open set is empty do below
            if len(open_set) == 0:
                print("No more node to visit")
                break
            #if open set is not empty, do below
            # remove current from open_set and get its index . Note, the list uses FIFO
            current = open_set.pop(list(open_set.keys())[0])
            c_id = self.calc_grid_index(current)

            #add explored node to closed_set (i.e the list of already visited nodes)
            closed_set[c_id] = current

            # to display the agent's movement on the screen
            if show_animation: 
                plt.scatter(self.calc_grid_position(current.x, self.min_x),
                            self.calc_grid_position(current.y, self.min_y),
                             s=300, c='yellow', marker='s')
                plt.pause(pause_time)       

                # To stop simulated display by pressing the  escape key...
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event:
                                             [exit(0) if event.key == 'escape'
                                              else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)
                
            # final path things    
            if len(closed_set) >= 2:
                locx, locy = self.calc_final_path(current, closed_set)
                
                for i in range(len(locx)-1):
                    px = (locx[i], locx[i+1])
                    py = (locy[i], locy[i+1])
                    if show_animation:
                        plt.plot(px, py, "-m", linewidth=4)
                        plt.pause(pause_time) 

            # To check if agent has arrived at goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Eureka, Goal Found!")
                if show_animation:
                    plt.scatter(current.x, current.y, s=300, c='orange', marker='s') 
                    plt.pause(pause_time)

                
                if len(closed_set) >= 2:
                    locx, locy = self.calc_final_path(current, closed_set)                
                    for i in range(len(locx)-1):
                        px = (locx[i], locx[i+1])
                        py = (locy[i], locy[i+1])
                        if show_animation:
                            plt.plot(px, py, "-m", linewidth=4)
                            plt.pause(pause_time)

                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                
                print("Total cost is: ", goal_node.cost )
                
                break

            if choice == 2:
                # expand_grid search grid based on motion model
                for i, _ in enumerate(self.motion):
                    node = self.Node(current.x + self.motion[i][0],
                                    current.y + self.motion[i][1],
                                    current.cost + self.motion[i][2], c_id, None)
                    n_id = self.calc_grid_index(node)

                    # verify if node is safe to go to or if its too close to obstacle
                    if not self.verify_node(node):
                        if show_animation:
                            plt.scatter(node.x,
                                    node.y,
                                    s=100, c='yellow', marker='s')
                            plt.pause(pause_time)                    
                        continue
                    
                
                    node.parent = current
                    open_set[n_id] = node
                    if show_animation:
                        plt.scatter(node.x,
                                    node.y,
                                    s=100, c='blue', marker='s')
                        plt.pause(pause_time)  


            elif choice ==1:

                for i, _ in enumerate(self.motion):
                    node = self.Node(current.x + self.motion[i][0],
                                    current.y + self.motion[i][1],
                                    current.cost + self.motion[i][2], c_id, None)
                    n_id = self.calc_grid_index(node)

                    # verify if node is safe to go to or if its too close to obstacle
                    if not self.verify_node(node):
                        if show_animation:
                            plt.scatter(node.x,
                                    node.y,
                                    s=100, c='yellow', marker='s')
                            plt.pause(pause_time)                    
                        continue
                    
                    # a little loop detection necessary for graph search
                    if (n_id not in closed_set) and (n_id not in open_set):
                        node.parent = current
                        open_set[n_id] = node
                        if show_animation:
                            plt.scatter(node.x,
                                    node.y,
                                    s=100, c='blue', marker='s')
                            plt.pause(pause_time)              
        # results of planning method calling another method to calculate final path
        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    # method to calculate final path        
    def calc_final_path(self, goal_node, closedset):
        # generate two arrays rx, ry that defines the path
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        n = closedset[goal_node.parent_index]
        
        #get indexes from the goal node to start node
        while n is not None:
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            n = n.parent

        return rx, ry

    # method to get coordinates using indexes on the grid system
    def calc_grid_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    # method to calculate index of x and y
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    # method to calculate grid index
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.xwidth + (node.x - self.min_x)

    #method to verify if node is safe for agent to visit
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

        # to check for possible collision
        if self.obmap[node.x][node.y]:
            return False

        return True

    #for obstacles...
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

        self.xwidth = round((self.maxx - self.min_x) / self.resolution)
        self.ywidth = round((self.maxy - self.min_y) / self.resolution)


        # genarating obstacles on map
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

    
    # defining an 8 directional motion model
    @staticmethod
    def get_motion_model_8n():

        motion =[[1, 0, 1],
                [1, -1, math.sqrt(2)],
                [0, -1, 1],
                [-1, -1,  math.sqrt(2)],
                [-1, 0, 1],
                [-1, 1,  math.sqrt(2)],
                [0, 1, 1],
                [1, 1,  math.sqrt(2)]
                ]
        return motion
