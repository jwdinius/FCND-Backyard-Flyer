import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection, pos_tol=1.0, box_length=10.0):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.pos_tol = pos_tol  # position tolerance for declaring success
        self.box_length = box_length  # side length of box
        self.start_local = None 
        self.end_local = None 

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            if abs(self.local_position[2]) < 1.03*self.target_position[2] \
                    and abs(self.local_position[2]) > 0.97*self.target_position[2]:
                # if near the desired height, prepare for box pattern by
                # computing waypoints and transition
                # TAKEOFF->WAYPOINT
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
                return
        elif self.flight_state == States.WAYPOINT:
            # on each position update, check
            # (1) are there any waypoints left?
            # (2) if there are not, and we are almost at rest,
            #     transition to landing
            pos = self.local_position[:2]  # don't include z comp
            #print("x: {}, y: {}, z: {}".format(pos[0], pos[1], pos[2]))
            tgt_pos = self.target_position[:2]
            ds = np.linalg.norm(tgt_pos - pos)
            if self.all_waypoints:
                if ds < self.pos_tol:
                    # WAYPOINT -> WAYPOINT
                    self.waypoint_transition()
            else:
                v = np.linalg.norm(self.local_velocity)
                if v < 0.05:
                    # WAYPOINT -> LANDING
                    self.landing_transition()


    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # this callback only needs to check for speed being small enough to transition from 
        # landing to disarm
        if self.flight_state == States.LANDING:
            curr_z = self.global_position[2]
            final_z = self.global_home[2]
            if abs(curr_z - final_z) < 0.05:
                # LANDING -> DISARMING
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                # MANUAL -> ARMING
                self.arming_transition()
                return
            if self.flight_state == States.ARMING:
                # ARMING -> TAKEOFF
                self.takeoff_transition()
                return
            if self.flight_state == States.DISARMING:
                # check that motors are not armed and that we are not in guided mode
                # before transitioning to MANUAL
                if not self.armed and not self.guided:
                    self.manual_transition()
                    return

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        tgt_xy = self.box_length
        tgt_z = self.target_position[2]
        curr_x, curr_y, _ = self.local_position  # don't need the curr_z value
        #print("####### {}".format(tgt_z))
        return [[curr_x + tgt_xy, curr_y, tgt_z],
                [curr_x + tgt_xy, curr_y + tgt_xy, tgt_z],
                [curr_x, curr_y + tgt_xy, tgt_z],
                [curr_x, curr_y, tgt_z]]
        

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()  # step 1
        self.arm()  # step 2
        # step 3
        lng, lat, alt = self.global_position
        self.set_home_position(lng, lat, alt)
        # step 4
        self.flight_state = States.ARMING
        self.start_local = self.local_position


    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        tgt_alt = 3.0  # step 1
        self.target_position[2] = tgt_alt  # this is in NED, and D=-alt
        self.takeoff(tgt_alt)  # step 2
        self.flight_state = States.TAKEOFF  # step 3

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")
        # pop the next waypoint when this method is called
        self.target_position = self.all_waypoints.pop(0)
        tgt_x, tgt_y, tgt_z = self.target_position
        self.cmd_position(tgt_x, tgt_y, tgt_z, 0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")
        self.end_local = self.local_position
        ds = np.linalg.norm(self.end_local - self.start_local)
        print("Final position error (required < 1.0m): {}m".format(ds))
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
