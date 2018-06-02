import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path, a_star_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
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
            DEADBAND = 1.0
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < DEADBAND:
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
                    g_goal = (-122.400915, 37.794507, 0)  # commercial/battery st. (lon, lat, alt)
                    self.plan_path(g_goal)
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
        # cmd_position wants local NED coordinates in meters, i.e. in relation to home
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

    def plan_path(self, g_goal):
        """
        Plan a path from current position to specified global position (lat, lon, alt)

        :param g_goal: goal, geodetic coordinates
        :return:
        """
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0, coordinates of the map center
        filename = 'colliders.csv'
        with open(filename) as f:
            for line in f:
                break
        (_, lat0, _, lon0) = line.split()
        lat0 = float(lat0.strip(','))
        lon0 = float(lon0.strip(','))

        # set home position to center of the map
        self.set_home_position(lon0, lat0, 0)

        # retrieve current global position of the Drone. (lon, lat, alt) as np.array
        g_pos = self.global_position

        # convert current global coordinates to NED frame (centered at home)
        l_pos = global_to_local(g_pos, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # Read in obstacle map
        map_data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # Define a grid for a particular altitude and safety margin around obstacles
        # the way create_grid works is it discretizes the map at 1meter resolution
        grid, north_offset, east_offset = create_grid(map_data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Convert NED position to point on the grid
        def int_round(i):
            return int(round(i))

        grid_start = (-north_offset + int_round(l_pos[0]), -east_offset + int_round(l_pos[1]))

        # Set goal in grid coordinates
        l_goal = global_to_local(g_goal, self.global_home)
        grid_goal = (-north_offset + int_round(l_goal[0]), -east_offset + int_round(l_goal[1]))

        # Run A* to find a path from start to goal on a graph using Voronoi regions
        print('Local Start and Goal: ', grid_start, grid_goal)
        starttime = time.time()

        path, _ = a_star_graph(map_data, TARGET_ALTITUDE, SAFETY_DISTANCE, grid_start, grid_goal)

        print('planned in {} secs. path length: {}'.format(time.time() - starttime, len(path)))

        path = prune_path(path)
        print('pruned path length: {}'.format(len(path)))

        # Convert path to waypoints
        HEADING = 0
        waypoints = [[int_round(p[0]) + north_offset, int_round(p[1]) + east_offset, TARGET_ALTITUDE, HEADING] for p in
                     path]
        # Set self.waypoints to follow
        self.waypoints = waypoints
        # send waypoints to sim for visualization
        self.send_waypoints()

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
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
