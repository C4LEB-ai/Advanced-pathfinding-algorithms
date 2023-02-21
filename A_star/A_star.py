
import time
import math
import json

import numpy as np
import pandas as pd
import tracemalloc
import matplotlib.pyplot as plt

show_animation = True
pause_time = 0.1

choice = int(input("select the operation you want to perform.\n1)A* TREE\n2)A* GRAPH\n"))

# Creating a class for A* Search
class AStarPlanner:

    def __init__(self, ox, oy, resolution, robot_radius, model = '8n'):
        """
        when called, function will require the following:
        ox, oy - the x and y cordinates for obstacles
        rosolution - Map's resolution for display
        robotRadius: Radius of the robot
        """

        self.resolution = resolution
        self.robot_radius= robot_radius
        self.min_x, self.min_y = 0, 0     
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0

        #choosing motion model...
        self.motion = self.get_motion_model_8n()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of the node on the grid-map
            self.y = y  # index of the node on the grid-map
            self.cost = cost # cost of getting to this node
            self.parent_index = parent_index # the index of the parent node

        # return a string containing the parameters of the node
        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):    
        """
        Path planning for A* search 
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
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        #plot start node and goal node on map specifying their cell colour
        if show_animation:
            plt.scatter(start_node.x, start_node.y, s=400, c='red', marker='s')
            plt.scatter(goal_node.x, goal_node.y, s=400, c='green', marker='s')

        #Create 2 dictionaries, open and closed set, add start node to open set
        open_set, closed_set = dict(), dict()    # 'open_set' stores nodes to expand (by using motion model), 'closed_set' stores nodes visited 
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            #if open set is empty do below
            if len(open_set) == 0:
                print("Open set is empty..")
                break


           

            # we use Manhattan distance for heuristic and calculate it here
            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic_m(goal_node,
                                                                     open_set[
                                                                         o]))
            #add explored node to closed_set (i.e the list of already visited nodes)
            current = open_set[c_id]

            # to print node id, cost, and heuristic
            print("Current node ID is {ID}, its path cost is {PathCost}, and its heuristic is {Heuristic}".format(ID=c_id, PathCost=round(current.cost,1), Heuristic=self.calc_heuristic_m(goal_node, current)))

            # to display the agent's movement on the screen
            if show_animation:  
                plt.scatter(self.calc_grid_position(current.x, self.min_x),
                            self.calc_grid_position(current.y, self.min_y),
                             s=300, c='yellow', marker='s')
                plt.pause(pause_time)                 


                # for stopping animation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # To check if agent has arrived at goal
            if current.x == goal_node.x and current.y == goal_node.y:  
                print("Eureka, Goal Found!")
                if show_animation:
                    plt.scatter(current.x,
                            current.y,
                            s=100, c='green', marker='s') 
                    plt.pause(pause_time)
                
                locx, locy = self.calc_final_path(current, closed_set)
                if len(locx)>=2:
                    for i in range(len(locx)-1):
                        px = (locx[i], locx[i+1])
                        py = (locy[i], locy[i+1])
                        if show_animation:
                            plt.plot(px, py, "-y", linewidth=4)
                            plt.pause(pause_time)    

                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                print("Total cost is :" ,goal_node.cost)
                break

            # Remove the item from the open set explicitly using its node id as A* doesnt use simpler LIFO or FIFO
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # for visualization, we mark visited nodes with red
            if show_animation:
                plt.scatter(current.x,
                        current.y,
                        s=100, c='red', marker='s') 
                plt.pause(pause_time)             

            # final path things
            locx, locy = self.calc_final_path(current, closed_set)
            if len(locx)>=2:
                for i in range(len(locx)-1):
                    px = (locx[i], locx[i+1])
                    py = (locy[i], locy[i+1])
                    if show_animation:
                        plt.plot(px, py, "-m", linewidth=4)
                        plt.pause(pause_time)
            
            #generate new node using motion model but dont check if its goal
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)  
##

                n_id = self.calc_grid_index(node)
                # expanding nodes using the motion model...# 
                if show_animation:
                    plt.scatter(node.x,
                            node.y,
                            s=100, c='yellow', marker='s')
                    plt.pause(pause_time)                  

                if n_id in closed_set:  
                # We mark cells that are visited already in red
                    if show_animation:
                        plt.scatter(node.x,
                                node.y,
                                s=100, c='red', marker='s')
                        plt.pause(pause_time)                       

                    continue

                # verify if node is safe to go to or if its too close to obstacle
                if not self.verify_node(node):
                    if show_animation:
                        plt.scatter(node.x,
                                node.y,
                                s=100, c='black', marker='s')
                        plt.pause(pause_time)                 
                    continue

                #creating condition for the graph or tree search using choice
                if choice == 1: # tree search
                    open_set[n_id] = node 
                    if show_animation:
                        plt.scatter(node.x,
                                node.y,
                                s=300, c='blue', marker='s')
                        plt.pause(pause_time)
                elif choice == 2: # graph search
                    if n_id not in open_set:   
                        open_set[n_id] = node  
                        if show_animation:
                            plt.scatter(node.x,
                                    node.y,
                                    s=300, c='blue', marker='s')
                            plt.pause(pause_time)
                    else:
                        if open_set[n_id].cost > node.cost:  
                            open_set[n_id] = node
                            if show_animation:
                                plt.scatter(node.x,
                                        node.y,
                                        s=300, c='green', marker='s')
                                plt.pause(pause_time)  

        # results of planning method calling another method to calculate final path
        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    # method to calculate final path
    def calc_final_path(self, goal_node, closed_set):
        # generate two arrays rx, ry that defines the path
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        #get indexes from the found goal node to start node
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    # cmethod to calculate the value of heuristic using Pythagorean Theorem - Euclidean Heuristic
    def calc_heuristic_e(n1, n2):
        # weight of the heuristic
        w = 1
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    @staticmethod
    ## calculate heuristic value by using Pythagoras Theorem - Euclidean Heuristic
    def calc_heuristic_m(n1, n2):
        # weight of the heuristic
        w = 1
        d = w * (math.fabs(n1.x - n2.x) + math.fabs(n1.y - n2.y))
        return d

    # methot to get coordinates using indexes on the grid system
    def calc_grid_position(self, index, min_position):

        pos = index * self.resolution + min_position
        return pos

   # method to calculate index of x and y
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    # method to calculate grid index
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    #method to verify if node is safe for agent to visit
    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    #for obstacles
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))


        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
 
        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d < self.robot_radius:      # This decides whether the agent can move along cells that are next to obstacles or walls. With '=', agent cannot move in these cells, without '=', the agent can move along these cells.
                        self.obstacle_map[ix][iy] = True
                        break

    
    @staticmethod
    def get_motion_model_8n():
       
        motion =[[-1, -1, math.sqrt(2)],
                [-1, 0, 1],
                [-1, 1, math.sqrt(2)],
                [0, 1, 1],
                [1, 1, math.sqrt(2)],
                [1, 0, 1],
                [1, -1, math.sqrt(2)],
                [0, -1, 1]
                ]
        return motion
   
    