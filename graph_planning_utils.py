import numpy as np
import networkx as nx
import numpy.linalg as LA

from scipy.spatial import Voronoi, voronoi_plot_2d
from queue import PriorityQueue
from bresenham import bresenham
from planning_utils import create_grid, coord_to_grid, grid_to_coord, heuristic
from udacidrone.frame_utils import global_to_local, local_to_global

import matplotlib.pyplot as plt

def a_star_graph(graph, h, start, goal):
    """
    Modified A* to work with NetworkX graphs.
    """
    # print(f"a_star:  start = {start}, goal = {goal}")

    start_node = find_nearest(graph, start)
    goal_node = find_nearest(graph, goal)

    # print(f"a_star:  start_node = {start_node}, goal_node = {goal_node}")

    queue = PriorityQueue()
    queue.put((0, start_node))
    visited = set(start_node)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start_node:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]
            
        if current_node == goal_node:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:                
                if next_node not in visited:                
                    cost = graph.edges[current_node, next_node]['weight']
                    branch_cost = current_cost + cost
                    queue_cost = branch_cost + h(next_node, goal_node)

                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))


    path = []
    path_cost = 0       

    if found:
        # retrace steps
        n = goal_node
        path_cost = branch[n][0]
        path.append(goal_node)
        while branch[n][1] != start_node:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost

def build_graph_from_edges(edges):
    """
    Create a graph from the `edges`
    """
    G = nx.Graph()
    for e in edges:
        p1 = e[0]
        p2 = e[1]
        # p1 = (int(round(e[0][0])), int(round(e[0][1])))
        # p2 = (int(round(e[1][0])), int(round(e[1][1])))
        dist = LA.norm(np.array(p2) - np.array(p1))
        G.add_edge(p1, p2, weight=dist)

    return G

def build_graph(grid, centers, north_offset, east_offset):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """
    # location of obstacle centres
    voronoi_graph = Voronoi(centers)

    # print(f"VORONOI:   num ridges: {len(voronoi_graph.ridge_vertices)}")

    # TODO: check each edge from voronoi_graph.ridge_vertices for collision
    edges = []
    for v in voronoi_graph.ridge_vertices:
        p1 = voronoi_graph.vertices[v[0]]
        p2 = voronoi_graph.vertices[v[1]]
        # p1 = (int(round(v1[0] - north_offset)), int(round(v1[1] - east_offset)))
        # p2 = (int(round(v2[0] - north_offset)), int(round(v2[1] - east_offset)))

        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    # print(f"VORONOI:   num edges: {len(edges)}")

    nx_graph = build_graph_from_edges(edges)

    print(nx_graph)

    return nx_graph    

def find_nearest(graph, pt):

    nearest_pt = None 
    nearest_dist = float("inf")

    for n in graph.nodes:
        dist = LA.norm(np.array(n) - np.array((pt[0], pt[1])))
        if dist < nearest_dist:
            nearest_dist = dist
            nearest_pt = n 
    
    return nearest_pt



