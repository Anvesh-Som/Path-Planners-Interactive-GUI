# Dijkstra path planning.
# This algorithm is named after Edsger W. Dijkstra, 1930-2002.
# See: http://en.wikipedia.org/wiki/Edsger_W._Dijkstra.
#
# Make sure you have installed all the required libraries (see the extra
# document with the installation instructions).
#
# First version: most simple implementation (not using a heap).
# pp_01_a_dijkstra
# (c) Claus Brenner, 15 JAN 2014
from turtle import shape
import numpy as np
import traceback
import gui
import common
import pp_01
import os
#import ptvsd
# The world extents in units.
world_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_nodes = None

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    dj.fill_obstacles(world_obstacles)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)
def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    dj.fill_obstacles(world_obstacles)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)

def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    dj.fill_obstacles(world_obstacles)
    update_callback()

def update_callback(pos = None):
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start==None or goal==None):
        global optimal_path
        global visited_nodes
        try:
            
            dj.fill_start_goal(int(start[0]), int(start[1]), int(goal[0]), int(goal[1]))
            dj.compute()
            C = np.empty([world_extents[0]*world_extents[1]], dtype=np.double)
            dj.return_visited(C)
            visited_nodes = np.reshape(C, (world_extents[0], world_extents[1]))
            d1_vsisted = dj.get_path_dim1()
            d2_visited = dj.get_path_dim2()
            X = np.empty([d1_vsisted*d2_visited], dtype=np.uint32)
            dj.return_path(X)
            
            optimal_path = []
            op_path = np.array(optimal_path,dtype=np.uint32)
            op_path = np.reshape(X, (d1_vsisted, d2_visited))
            optimal_path = [tuple(row) for row in op_path]
            #print(optimal_path)

            #optimal_path, visited_nodes = dijkstra(start, goal, world_obstacles)
        except Exception as e:
            print (traceback.print_exc())
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)

if __name__ == '__main__':
    # Link functions to buttons.

    port=3000
    #ptvsd.enable_attach(address =('127.0.0.1', port))
    #ptvsd.wait_for_attach()
    callbacks = {"update": update_callback,
                 "button_1_press": add_obstacle,
                 "button_1_drag": add_obstacle,
                 "button_1_release": update_callback,
                 "button_2_press": remove_obstacle,
                 "button_2_drag": remove_obstacle,
                 "button_2_release": update_callback,
                 "button_3_press": remove_obstacle,
                 "button_3_drag": remove_obstacle,
                 "button_3_release": update_callback,
                 }
    # Extra buttons.
    buttons = [("Clear", clear_obstacles)]
    dj = pp_01.dijkstra(world_extents[0], world_extents[1])
    # Init GUI.
    gui = gui.GUI(world_extents, 4, callbacks,
                  buttons, "on", "Simple Dijkstra Algorithm.")

    # Start GUI main loop.
    gui.run()
