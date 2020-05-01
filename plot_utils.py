import numpy as np
import matplotlib

def draw_obstacles(data, grid, ax):

    north_min = np.amin(data[:, 0] - data[:, 3])
    north_max = np.amax(data[:, 0] + data[:, 3])
    east_min = np.amin(data[:, 1] - data[:, 4])
    east_max = np.amax(data[:, 1] + data[:, 4])

    north_extent = int(np.ceil(north_max - north_min))
    east_extent = int(np.ceil(east_max - east_min))            
    north_offset = int(np.floor(north_min))
    east_offset = int(np.floor(east_min))

    x_min = east_offset
    x_max = east_offset + east_extent
    y_min = north_offset
    y_max = y_min + north_extent

    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","lightgrey","grey","dimgrey"])
    ax.imshow(grid, cmap=cmap, origin='lower', extent=[x_min, x_max, y_min, y_max])

    ax.set_ylabel('north')
    ax.set_xlabel('east')

def draw_path(path, ax, north_offset, east_offset):
    nodes = np.array(path)
    nodes += (north_offset, east_offset)
    ax.plot(nodes[:, 1], nodes[:, 0], color='red', linewidth=0.5)
    ax.plot(nodes[:, 1], nodes[:, 0], color='black', marker='o', markersize=2)

def draw_endpoints(ax, start, goal, north_offset, east_offset):
    ax.plot(start[1] + east_offset, start[0] + north_offset, marker='x', markersize=6, color="orangered")
    ax.plot(goal[1] + east_offset, goal[0] + north_offset, marker='x', markersize=6, color="orangered")    

def draw_graph(graph, ax, north_offset, east_offset):
    for edge in graph.edges:
        p1 = edge[0]
        p2 = edge[1]
        ax.plot([p1[1] + east_offset, p2[1] + east_offset], [p1[0] + north_offset, p2[0] + north_offset], linewidth=1, color="lime")    