#!/usr/bin/env python

import pygame
from RRT_base import RRT_Map
from RRT_base import RRT_Tree
import time

def main():

    #init the variables
    MapDim = (1000,600)
    start = (50,50)
    goal = (900,510)
    ObsDim = 0
    ObsNum = 0
    t1 = 0

    #init the map and the tree
    pygame.init()
    map = RRT_Map(start, goal, MapDim, ObsDim, ObsNum)
    tree = RRT_Tree(start, goal, MapDim, ObsDim, ObsNum)

    #gen and draw the obstacles
    obstacles = tree.gen_obstacle()
    map.drawMap(obstacles)   

    #gen the tree until goal region is reached
    i = 0
    t1=time.time()
    while not tree.path_to_goal():
        #check for time-out first
        dt = time.time() - t1
        if dt > 10:
            raise

        if i % 10 == 0:  #10% bais towards the goal
            x , y, parent = tree.bias_2_goal() 
            #x,y are coords of child node which id is -1, parent is id of parent node
            pygame.draw.circle(map.map, map.grey, (x[-1],y[-1]), map.node_rad, 0)
            pygame.draw.line(map.map, map.blue, (x[-1],y[-1]), (x[parent[-1]],y[parent[-1]]), map.edge_thick)
        else:
            x, y, parent = tree.expand()
            pygame.draw.circle(map.map, map.grey, (x[-1],y[-1]), map.node_rad, map.node_thick, 0)
            pygame.draw.line(map.map, map.blue, (x[-1],y[-1]), (x[parent[-1]],y[parent[-1]]), map.edge_thick)

        #update the display each 5 iterations
        if i % 5 == 0 : 
            pygame.display.update()
        i += 1
    
    #draw the path
    map.drawPath(tree.path_coords())
    print(f"\n Number of Obstacles = {ObsNum}")
    print(f"\n Side Length of every Obstacles = {ObsDim} \n")

    
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False
