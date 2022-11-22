#!/usr/bin/env python
from heapq import heappop, heappush
from math import *
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d


# Define RB5 Available Motions
movements = [(0, -1), (0, 1), (-1, 0), (1, 0),                  # Drive Up, Drive Down, Slide Left, Slide Right
             (-1, -1), (-1, 1), (1, -1), (1, 1)]                # Diagonal Movements (Top Left, Bottom Left, Top Right, Bottom Right)

# Define A* path planner class
class A_Star():
    '''
    A* Path Planner Class
    '''
    def __init__(self, start, goal, tol, map, max_iters = 10000):
        '''
        Initialize parameters
        '''
        self.start = start                                      # Starting Point (x, y)
        self.goal = goal                                        # Ending Point (x, y)
        self.tol = tol                                          # Error Tolerance
        self.map = map                                          # 2D Map
        self.map_width, self.map_height = np.shape(map)         # Map Size
        self.max_iters = max_iters                              # Maximum Iterations
        
        self.visited = []                                       # List that stores visited nodes
        self.discovered = []                                    # List that stores discovered nodes
        self.path = []                                          # List that store path nodes
        
        # Add to discovered queue {(cost + euclid_dist, (curr_x, curr_y, [path], cost + delta))}
        heappush(self.discovered, (self.euclid_dist(start, goal), (start, [], 0)))
        
    def euclid_dist(self, pt_init, pt_final):
        '''
        Calculate Euclidean distance between points
        '''
        x_init, y_init = pt_init                                # Initial point (x, y)
        x_final, y_final = pt_final                             # Final point (x, y)
        return sqrt(pow(x_final - x_init, 2) + pow(y_final - y_init, 2))
    
    def check_map(self, pos):
        '''
        Check map for non-diagonal movement
        '''
        x, y = pos                                              # Position (x, y)
        # Check whether x, y exceed map limit
        if 0 <= x < self.map_width and 0 <= y < self.map_height:
            return self.map[x, y] == 0                          # Free space
        
    def check_map_diag(self, pos, movement):
        '''
        Check map for diagonal movement
        '''
        x, y = pos                                              # Position (x, y)
        dx, dy = movement                                       # Movement (dx, dy)
        x_new, y_new = x + dx, y + dy                           # Updated position (x_new, y_new)
        # Check whether the new positions are valid
        return (self.check_map([x_new, y]) and self.check_map([x, y_new]) and self.check_map([x_new, y_new]))
    
    def check_goal(self, pos):
        '''
        Check if RB5 arrives at the goal
        '''
        return self.euclid_dist(pos, self.goal) <= self.tol
    
    def update_pos(self, pos, movement):
        '''
        Update RB5 position
        '''
        x, y = pos                                              # Position (x, y)
        dx, dy = movement                                       # Movement (dx, dy)
        return (x + dx, y + dy)
    
    def plan_path(self):
        '''
        Plan path with given starting point and ending point
        '''
        count = 0                                               # Initialize counter
        while count < self.max_iters:
            
            # Pop discovered node queue
            curr_info = heappop(self.discovered)
            curr_pos, path, cost = curr_info[1]
            
            # Check if RB5 reaches its goal
            if self.check_goal(curr_pos):
                self.path = path
                print("[MESSAGE] A* Path Planner Found a Path.")
                break
            
            # Check if the node is visited
            if curr_pos in self.visited:
                continue
            
            # Append to visited node
            self.visited.append(curr_pos)
            
            # Explore other nodes
            for movement in movements:
                new_pos = self.update_pos(curr_pos, movement)
                # If diagonal movement
                if abs(movement[0]) == abs(movement[1]):
                    # Check if diagonal movement is valid
                    if self.check_map_diag(curr_pos, movement):
                        # Add new node information to the discovered nodes list
                        euclidean_dist = self.euclid_dist(new_pos, self.goal)
                        total_cost = cost + euclidean_dist
                        heappush(self.discovered, (total_cost, (new_pos, path + [movement], cost +1)))
                else:
                    # Check if non-diagonal movement is valid
                    if self.check_map(new_pos):
                        # Add new node information to the discovered nodes list
                        euclidean_dist = self.euclid_dist(new_pos, self.goal)
                        total_cost = cost + euclidean_dist
                        heappush(self.discovered, (total_cost, (new_pos, path + [movement], cost +1)))
            
            # Increment counter
            count += 1
        
        # Path planner failed to find a path
        if count == self.max_iters: 
            print("[MESSAGE] A* Path Planner Failed to Find a Path.")
            
            
# Define Voronoi path planner class
class voronoi():
    '''
    Voronoi Path Planner Class
    '''
    def __init__(self, start, goal, tol, map, max_iters = 10000, verbose=False):
        '''
        Initialize parameters
        '''
        self.start = start                                      # Starting Point (x, y)
        self.goal = goal                                        # Ending Point (x, y)
        self.tol = tol                                          # Error Tolerance
        self.map = map                                          # 2D Map
        self.max_iters = max_iters                              # Maximum Iterations
        self.verbose = verbose                                  # Visual Display
        
        self.build_Voronoi()                                    # Compute Voronoi Graph
        
    def check_obstacle(self, height, width):
        '''
        Check if the node is an obstacle
        '''
        return self.map[height, width] == 1
    
    def check_freespace(self, height, width):
        '''
        Check if the node is a freespace
        '''
        return not self.check_obstacle(height, width)
        
    def build_Voronoi(self):
        '''
        Build Voronoi class object
        '''
        # Get map size
        map_height, map_width = self.map.shape 
        # Initialize a set that stores the Voronoi nodes that are a freespace
        nodes = set()
        for h in range(1, map_height - 1):
            for w in range(1, map_width - 1):
                if self.check_obstacle(h, w):
                    if self.check_freespace(h - 1, w):
                        nodes.add((w, h - 1))
                    if self.check_freespace(h + 1, w):
                        nodes.add((w, h + 1))
                    if self.check_freespace(h, w - 1):
                        nodes.add((w - 1, h))
                    if self.check_freespace(h, w + 1):
                        nodes.add((w + 1, h))
                        
        # Convert to array from set
        nodes = np.array(list(nodes)).reshape((-1, 2))
        self.voronoi = Voronoi(nodes)
        
        # Display result if true
        if self.verbose:
            fig = voronoi_plot_2d(self.voronoi)
            fig.set_size_inches(16, 16)
            fig.suptitle("Visualization of the Voronoi Graph")
            plt.show()
            
    def manhattan_dist(self, pt_init, pt_final):
        '''
        Calculate Manhattann Distance
        '''
        x_init, y_init = pt_init
        x_final, y_final = pt_final
        return max(abs(x_final - x_init), abs(y_final - y_init))
    
    def update_pos(self, pos, movement):
        '''
        Update RB5 position
        '''
        x, y = pos                                              # Position (x, y)
        dx, dy = movement                                       # Movement (dx, dy)
        return [x + dx, y + dy]
    
    def plan_path(self):
        '''
        Plan path with given starting point and ending point
        '''
        # Get the vertices
        vertices = self.voronoi.vertices.astype(int)
        # Initialize the nodes list (freespace only)
        nodes = []
        
        # Append node that is a freespace to the nodes list
        for vertex in vertices:
            # Get Vertex position (x, y)
            x, y = vertex
            if self.check_freespace(x, y):
                nodes.append(list(vertex))
                        
        # Initialize discovered and visited lists
        discovered, visited = [], []
        heappush(discovered, (0, (self.start, [], 0)))
        
        # Initialize counter
        count = 0

        while discovered and count <= self.max_iters:
            
            # Pop discovered node queue
            curr_pos, path, cost = heappop(discovered)[1]
            
            # Check if the node is visited
            if curr_pos in visited:
                continue
            
            # Append to visited node list
            visited.append(curr_pos)
            
            # Check if RB5 reaches its goal
            if self.manhattan_dist(curr_pos, self.goal) <= self.tol:
                print ('[MESSAGE] Voronoi Path Planner Found a Path.')
                return path
            
            # Explore other nodes
            for movement in movements:
                # Update position
                new_pos = self.update_pos(curr_pos, movement)
                # If the updated position is in the Voronoi node list
                if new_pos in nodes:
                    # Add new node information to the discovered nodes list
                    manhattan_Dist = self.manhattan_dist(new_pos, self.goal)
                    total_cost = cost + manhattan_Dist
                    heappush(discovered, (total_cost, (new_pos, path + [movement], cost + 1)))
            
            # Increment counter
            count += 1
            
        # Path planner failed to find a path
        if count == self.max_iters: 
            print("[MESSAGE] Voronois Path Planner Failed to Find a Path.")
    
        
        


                        
            
            
    
        

