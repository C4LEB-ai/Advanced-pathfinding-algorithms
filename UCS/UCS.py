#importing required parkages
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import json
import tracemalloc
import time


show_animation = True

pause_time = 0.1

choice = int(input("select the operation you want to perform.\n1)UCS TREE\n2)UCS GRAPH\n"))

# Creating a class for UCS Search
class UCSplanner:

    def __init__(self, ox, oy, resolution, robot_radius, model = '8n'):
        """
       when called, function will require the following:
       ox, oy - the x and y cordinates for obstacles
       rosolution - Map's resolution for display
       robotRadius: Radius of the robot
       """
       

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        
        #choosing motion model...
        self.motion = self.get_motion_model_8n()  

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
        Path planning for UCS search 
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
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while 1:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # to display the agent's movement on the screen
            if show_animation:  
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                plt.scatter(self.calc_position(current.x, self.min_x),	
                            self.calc_position(current.y, self.min_y),	
                             s=300, c='yellow', marker='s')	
                plt.pause(pause_time)   
                
                # To stop simulated display by pressing the  escape key...
                if show_animation: 
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
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
                if len(locx)>2:	
                    for i in range(len(locx)-1):	
                        px = (locx[i], locx[i+1])	
                        py = (locy[i], locy[i+1])	
                        if show_animation: 
                            plt.plot(px, py, "-m", linewidth=4)	
                            plt.pause(pause_time)    
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                print("Total cost is: " ,goal_node.cost)
                break
            
            # Remove the item explicitly from the open set using its index since we are not using FIFO or LIFO
            del open_set[c_id]  

            # Add it to the closed set
            closed_set[c_id] = current

            # We mark cells that have already been visited in red
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

            #generate new node but dont check if its goal
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)   

                n_id = self.calc_index(node)

                # node exploration using chosen motion model
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
                if choice == 1:
                    open_set[n_id] = node  
                    if show_animation: 
                        plt.scatter(node.x,
                                node.y,
                                s=300, c='blue', marker='s')
                        plt.pause(pause_time)
                elif choice == 2:
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
                else:
                    print("Please select an option...")                        

                        

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    # method to calculate final path 
    def calc_final_path(self, goal_node, closed_set):
        # generate two arrays rx, ry that defines the path
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    #get coordinates from index in grid system
    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    # method to calculate index of x and y
    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    # method to calculate grid index
    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    #method to verify if node is safe for agent to visit
    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))


        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)


        # genarating obstacles on map
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break



    @staticmethod
    #defining the 8 directional motion model
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
    