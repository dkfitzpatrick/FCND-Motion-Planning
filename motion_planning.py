import argparse
import time
import msgpack
from enum import Enum, auto
import math
import numpy as np
import time

from planning_utils import create_grid, heuristic, home_latlon, coord_to_grid, grid_to_coord, prunepath, a_star_grid, calibrate_prunepath, prunepath_normalized
from graph_planning_utils import build_graph, a_star_graph
from plot_utils import draw_obstacles, draw_path, draw_endpoints, draw_graph

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global

import matplotlib.pyplot as plt

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

class MotionPlanning(Drone):

    def __init__(self, connection, goal_lon, goal_lat, alt, planner_type, prunepath_type, dump_plot):
        super().__init__(connection)

        self.global_goal = np.array([goal_lon, goal_lat, alt])
        self.goal_lon = goal_lon
        self.goal_lat = goal_lat
        self.alt = alt
        self.planner_type = planner_type
        self.prunepath_type = prunepath_type
        self.dump_plot = dump_plot
        self.target_position = np.array([0, 0, 0, 0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # [velnorth, veleast, veldown] = self.local_velocity
            # absv = math.sqrt(velnorth*velnorth + veleast*veleast)
            # print(f"velocity update: abs vel = {absv:5.2f}, north = {velnorth:5.2f}, east = {veleast:5.2f}, down = {veldown:5.2f}")
            waypoint_thresh = 1.0
            if len(self.waypoints) > 1:
                # calculate waypoint threshold based on next heading update
                next_heading = np.arctan2((self.waypoints[0][1] - self.target_position[1]), (self.waypoints[0][0] - self.target_position[0]))
                # if the next two waypoints are basically aligned, loosen up the tolerances
                # probably can do a heuristic based on what the velocity components are and the upcoming direction adjustment as well
                if abs(self.heading - next_heading) < 0.25:
                    waypoint_thresh = 5.0
                elif abs(self.heading - next_heading) < 0.40:
                    waypoint_thresh = 2.5

            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < waypoint_thresh:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.heading = np.arctan2((self.target_position[1] - self.local_position[1]), (self.target_position[0] - self.local_position[0]))
        self.target_position[3] = self.heading
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = home_latlon('colliders.csv')
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position (this was not clear)
        global_position = self.global_position
        print(f"Global Position: {self.global_position}")
        print(f"Global Home: {self.global_home}")
 
        # TODO: convert to current local position using global_to_local()
        local_north, local_east, _ = global_to_local(self.global_position, self.global_home)
        print(f"Local north: {local_north}, east: {local_east}")
        
        print(f"global home {self.global_home}, position {self.global_position}, local position {self.local_position}")
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, centers, north_offset, east_offset = create_grid(data, self.alt, SAFETY_DISTANCE)
        print(f"North offset = {north_offset}")
        print(f"East offset = {east_offset}")
        
        global_start = global_position
        local_start = self.local_position[:]
        grid_start = coord_to_grid(local_start, north_offset, east_offset)
        print(f"Global Start: {global_start}")
        print(f"Local Start: {local_start}")
        print(f"Grid Start: {grid_start}")

        # update start location
        self.target_position = np.array([local_start[0], local_start[1], self.alt, 0])

        global_goal = (self.global_goal[0], self.global_goal[1], self.alt) 
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = coord_to_grid(local_goal, north_offset, east_offset)
        print(f"Global Goal: {global_goal}")
        print(f"Local Goal: {local_goal}")
        print(f"Grid Goal: {grid_goal}")

        if self.planner_type == "grid":
            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            t_start = time.perf_counter()
            path, cost = a_star_grid(grid, heuristic, grid_start, grid_goal)
            t_elapsed = time.perf_counter() - t_start
            print(f"path (grid) generated.  length = {len(path)}, cost = {cost} in {t_elapsed} seconds.")

            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            ## path = prunepath(path, 1e-3)  # path prune
            if self.prunepath_type == "normalize":
                eps = calibrate_prunepath(0.1*SAFETY_DISTANCE)
                print(f"epsilon calibration: {eps}")
                path = prunepath_normalized(path, eps)            
            else:
                path = prunepath(path, 1e-2)

            print(f"path pruned.  new length = {len(path)}")
            print(path)

            if self.dump_plot:
                fig = plt.figure(figsize=(8,8))
                ax = fig.add_subplot(1,1,1)
                draw_obstacles(data, grid, ax)
                draw_path(path, ax, north_offset, east_offset)
                draw_endpoints(ax, grid_start, grid_goal, north_offset, east_offset)
                fig.savefig('route_grid.png', dpi=100)
        elif self.planner_type == "graph":           
            t_start = time.perf_counter()
            graph = build_graph(grid, centers, north_offset, east_offset)
            print(f"graph generated.   num edges = {len(graph.edges)}")
            # Run A* to find a path from start to goal
            path, cost = a_star_graph(graph, heuristic, grid_start, grid_goal)
            t_elapsed = time.perf_counter() - t_start
            print(f"path (graph) generated.  length = {len(path)}, cost = {cost} in {t_elapsed} seconds.")

            # TODO: prune path to minimize number of waypoints
            # path = prunepath(path, 1e-3)  # path prune
            if self.prunepath_type == "normalize":
                eps = calibrate_prunepath(0.1*SAFETY_DISTANCE)
                print(f"epsilon calibration: {eps}")
                path = prunepath_normalized(path, eps)            
                print(f"path pruned.  new length = {len(path)}")
            else:
                path = prune_path(path, 1e-2)

            if self.dump_plot:
                fig = plt.figure(figsize=(8,8))
                ax = fig.add_subplot(1,1,1)
                draw_obstacles(data, grid, ax)
                draw_graph(graph, ax, north_offset, east_offset)
                draw_path(path, ax, north_offset, east_offset)
                draw_endpoints(ax, grid_start, grid_goal, north_offset, east_offset)
                fig.savefig('route_graph.png', dpi=100)            
        else:
            exit(-1)

        if len(path) == 0:
            self.stop()
            return

        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]

        # Set self.waypoints
        self.waypoints = waypoints

        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
        print('path generation completed')

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    # default waypoint = 2nd and Market
    parser.add_argument('--goal_lon', type=float, default=-122.396685, help="Goal destination longitude")
    parser.add_argument('--goal_lat', type=float, default=37.797194, help="Goal destination latitude")
    parser.add_argument('--alt', type=float, default=TARGET_ALTITUDE, help="Travel altitude")
    parser.add_argument('--planner_type', type=str, default='grid', help="planner type to create waypoints")
    parser.add_argument('--prunepath_type', type=str, default='normalize', help="prune method for waypoints")
    parser.add_argument('--dump_plot', action="store_true", help="dump route plot")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=240)
    drone = MotionPlanning(conn, goal_lon=args.goal_lon, goal_lat=args.goal_lat, alt=args.alt, 
        planner_type=args.planner_type, prunepath_type=args.prunepath_type, dump_plot=args.dump_plot)
    time.sleep(1)

    drone.start()
