#!/usr/bin/env python


import math
import random
import pygame


#class of drawing methods

class RRT_Map():

    #config the window
    def __init__(self, start, goal, MapDim, ObsDim, ObsNum):
        self.start = start
        self.goal = goal
        self.mapH , self.mapW = MapDim

        #window settings
        pygame.display.set_caption('RRT Path Planning')
        self.map = pygame.display.set_mode((self.mapH, self.mapW))
        self.map.fill((255,255,255))

        #nodes and edges settings
        self.node_rad = 2
        self.node_thick = 0
        self.edge_thick = 1

        #obstacles settings
        self.obs = []
        self.obs_num = ObsNum
        self.obs_dim = ObsDim

        #color settings
        self.red = (255,0,0)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.white = (255,255,255)
        self.grey = (70,70,70)

    #draw the start and goal nodes & the obstacles
    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.green, self.start, self.node_rad+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal, self.node_rad+20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path_coords):
        for node_coords in path_coords:
            pygame.draw.circle(self.map, self.red, node_coords, self.node_rad+2, 0)

    #draw every generated obstacle
    def drawObs(self, obstacles):
        #obs_list = obstacles.copy()
        obs_list = obstacles
        while (len(obstacles)>0):
            pygame.draw.rect(self.map, self.grey, obs_list.pop(0))



#class of RRT algorithm methods

class RRT_Tree():

    #init the tree & main variables
    def __init__(self, start, goal, MapDim, ObsDim, ObsNum):
        self.start = start
        self.goal = goal
        self.mapH, self.mapW = MapDim

        self.goal_id = None
        self.goalFlag = False


        self.x = []
        self.y = []
        self.parent = []

        self.x.append(start[0])
        self.y.append(start[1])
        self.parent.append(0)

        self.obs = []
        self.obs_num = ObsNum
        self.obs_dim = ObsDim

        self.path = []

    #random rect represented by its upper left corner
    def gen_random_rect_coin(self):
        cx = int(random.uniform(0,self.mapH-self.obs_dim))
        cy = int(random.uniform(0,self.mapW-self.obs_dim))

        return (cx,cy)
    
    #check intersec between rect and circle, true if yes
    def intersec_check(self, rect, center):
        rect_center = rect.center
        distance = ((center[0] - rect_center[0])**2 + (center[1] - rect_center[1])**2)**0.5
        distance_max = 20 + max(rect.width, rect.height) / (math.sqrt(2))
        return distance <= distance_max

    #create obstacles that dont collide with start & goal
    def gen_obstacle(self):

        for i in range(self.obs_num):

            collide = True
            while collide:            
                coin = self.gen_random_rect_coin()
                rect = pygame.Rect(coin, (self.obs_dim, self.obs_dim))

                #function that checks if the circle collide with rect

                if not ( rect.collidepoint(self.start) ) and not ( self.intersec_check(rect=rect, center=self.goal) ):
                    collide = False

            self.obs.append(rect)

        obstacles = self.obs.copy()

        return obstacles

    #add a node in a specific tree placement
    def gen_node(self, id, x, y):
        self.x.insert(id, x)
        self.y.insert(id, y)

    #delete a node
    def del_node(self, id):
        self.x.pop(id)
        self.y.pop(id)

    #insert a link between parent node and child node
    def gen_edge(self, parent_id, child_id):
        self.parent.insert(child_id, parent_id)

    #delete a link
    def del_edge(self, child_id):
        self.parent.pop(child_id)

    #calculate the total nb of nodes in the tree
    def nb_nodes(self):
        return len(self.x)

    #calculate the distance between 2 nodes
    def distance(self, n1_id, n2_id):
        (x1,y1) = (self.x[n1_id],self.y[n1_id])
        (x2,y2) = (self.x[n2_id],self.y[n2_id])

        return math.sqrt( (float(x2)-float(x1))**2 + (float(y2)-float(y1))**2 )

    #generate a random sample
    def gen_random_sample(self):
        x_samp = int(random.uniform(0,self.mapH))
        y_samp = int(random.uniform(0,self.mapW))

        return x_samp,y_samp
    
    #check if added node (last one on the tree) is collision free from every obstacle
    def node_isFree(self):
        node_id = self.nb_nodes() - 1  #indexing starts from 0
        (x,y) = (self.x[node_id],self.y[node_id])

        obstacles = self.obs.copy()
        while(len(obstacles)>0):
            rect = obstacles.pop(0)
            if rect.collidepoint(x,y):
                self.del_node(node_id)
                return False
            
        return True


    #check if an edge is collision free from every obstacle
    def edge_isFree(self, id1, id2):
        (x1,y1) = (self.x[id1],self.y[id1])
        (x2,y2) = (self.x[id2],self.y[id2])
    
        obstacles = self.obs.copy()
        while(len(obstacles)>0):
            rect = obstacles.pop(0)

            for i in range(101):  #discretize the edge by a step of 0.01
                u = i/100
                xi = x2 - (x2-x1)*u
                yi = y2 - (y2-y1)*u

                if rect.collidepoint(xi,yi):
                    return False
                
        return True


    #connect two nodes if the edge is collision free
    def connect(self, id1, id2):
        if not self.edge_isFree(id1, id2):
            self.del_node(id2)
            return False #delete the child note and return false to indicate failure
        else:
            self.gen_edge(id1, id2)
            return True 


    #find the nearest node to the sample
    def find_nearest(self, sample_id):
        d_min = self.distance(0, sample_id) #at first suppose that start is nearest
        nearest_id = 0

        for i in range(0,sample_id):
            if self.distance(i, sample_id) <= d_min:
                d_min = self.distance(i, sample_id)
                nearest_id = i
        
        return nearest_id


    #create new node for fixed step size & verify the goal region
    def step_node(self, nearest_id, sample_id, step=20):
        d = self.distance(nearest_id, sample_id)
        if d>step:
            u = step/d #for later use

            (x_nearest,y_nearest) = (self.x[nearest_id],self.y[nearest_id])
            (x_sample,y_sample) = (self.x[sample_id],self.y[sample_id]) #(hyp.) sample alr added to tree
            theta = math.atan2(y_sample-y_nearest,x_sample-x_nearest)

            (x_new,y_new) = (int(x_nearest + step * math.cos(theta)),int(y_nearest + step * math.sin(theta)))
            new_id = sample_id
            self.del_node(sample_id)

            #if new node is within goal region, add the goal to the tree and end it
            if abs(x_new - self.goal[0])<20 and abs(y_new - self.goal[1])<20:
                self.goal_id = new_id
                self.gen_node(self.goal_id, self.goal[0], self.goal[1])
                self.goalFlag = True 
            #if new node is not within goal region, just add it to the tree as a node
            else:
                self.gen_node(new_id, x_new, y_new)
        
        #if d<step then the sample is alr added to the tree, no need to do anything

    #expand the tree randomly
    def expand(self):
        id = self.nb_nodes()
        x_samp, y_samp = self.gen_random_sample()
        self.gen_node(id, x_samp, y_samp)

        if self.node_isFree():
            nearest_id = self.find_nearest(id)
            self.step_node(nearest_id, id)
            self.connect(nearest_id, id)
        
        return self.x,self.y,self.parent
    

    #expand the tree towards the goal
    def bias_2_goal(self):
        #create goal as temporarly samp
        id = self.nb_nodes()
        self.gen_node(id, self.goal[0], self.goal[1])

        #find nearest and connect it to new /goal region already collision free
        nearest_id = self.find_nearest(id)
        self.step_node(nearest_id, id)
        self.connect(nearest_id, id)

        return self.x,self.y,self.parent

    #get path nodes id moving backward  
    def path_to_goal(self):
        if self.goalFlag:
            self.path.append(self.goal_id)
            parent_id = self.parent[self.goal_id]

            while (parent_id != 0):
                self.path.append(parent_id)
                parent_id = self.parent[parent_id]
            self.path.append(0)

        return self.goalFlag
    

    #get path nodes coordinates
    def path_coords(self):
        self.path_coords=[]
        for node_id in self.path:
            self.path_coords.append((self.x[node_id],self.y[node_id]))
        
        return self.path_coords
