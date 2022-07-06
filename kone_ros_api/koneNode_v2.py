#!/usr/bin/env python3
# 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

from numpy import empty, source
import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from kone_ros_api.koneAdaptor_v2 import koneAdaptor
from rmf_lift_msgs.msg import LiftState, LiftRequest
from rmf_fleet_msgs.msg import FleetState, RobotState, Location

import time
from threading import Thread
from pprint import pprint
import datetime

import yaml, os
from ament_index_python.packages import get_package_share_directory


class LiftNode(Node):
    def __init__(self, lift_name):
        self.node_name = f"{lift_name}_lift_node"
        super().__init__(self.node_name)

        # Get config from yaml
        self.config_yaml = self.get_config()

        self.prev_fleet_robot_level_dict = None
        self.fleet_robot_level_dict = None

        self.koneAdaptorGalen = koneAdaptor(self.config_yaml)

        # Var for storing previous session
        self.prev_rmf_lift_request = []
        for i in range (len(self.koneAdaptorGalen.liftNameList)):
            self.prev_rmf_lift_request.append(LiftRequest)


        self.lift_state_pub = self.create_publisher(LiftState, "lift_states", 10)
 
        self.lift_request_sub = self.create_subscription(
            LiftRequest, "lift_requests", self.liftRequestCallBack, 10
        )

        self.reset_liftstate_ws = True #open ws for the 1st time

        self.reset_liftstate_ws_timer = self.create_timer(5.0, self.reset_liftstate_ws_CallBack)
        self.liftstate_timer = self.create_timer(1.0, self.liftStatePublisherCallBack)
        
        threadKoneLiftstateWS = Thread(
            target=self.liftstate_websocket, name="koneliftstate", args=("koneliftstate",)
        )
        threadKoneLiftstateWS.daemon = True
        threadKoneLiftstateWS.start()


    def get_config(self):
        env_config = os.path.join(
            get_package_share_directory("kone_ros_api"), "config", "env.yaml"
        )
        with open(env_config, "r") as stream:
            try:
                parsed_yaml = yaml.safe_load(stream)
                # print(parsed_yaml)
                self.get_logger().info("Succcess reading env.yaml")
            except yaml.YAMLError as exc:
                # print(exc)
                self.get_logger().info("Failure reading env.yaml")
            return parsed_yaml


    def reset_liftstate_ws_CallBack(self):
        # check if liftstate websocket is inactive for more than 59s or ws opened for more than 300s, then restart it
        # max kone liftstate ws subscription duration is 300s (for kone api v2)
        current_timestamp = time.time()
        last_active_timestamp = self.koneAdaptorGalen.last_active_timestamp_for_liftstate_ws
        last_opened_timestamp = self.koneAdaptorGalen.last_active_timestamp_for_liftstate_ws_open
        time_elapsed_since_last_ws_active = current_timestamp - last_active_timestamp
        time_elapsed_since_last_ws_opened = current_timestamp - last_opened_timestamp
        
        if (time_elapsed_since_last_ws_active >= 55.0 or time_elapsed_since_last_ws_opened >= 295.0):
            print ("time_elapsed_since_last_ws_active: " + str(time_elapsed_since_last_ws_active))
            print ("time_elapsed_since_last_ws_opened: " + str(time_elapsed_since_last_ws_opened))
            self.reset_liftstate_ws = True
            self.koneAdaptorGalen.closeSocketMsg_state(0)

    def liftstate_websocket(self, adaptername):
        while True:
            if (self.reset_liftstate_ws == True):
                #reset ws
                self.reset_liftstate_ws = False
                self.koneAdaptorGalen.openLiftStateWS()
                self.koneAdaptorGalen.runSocketTilComplete_state()

    def liftStatePublisherCallBack(self):

        for i in range(len(self.koneAdaptorGalen.current_liftstate_list)):
            rmfLiftStateMsg = self.koneAdaptorGalen.current_liftstate_list[i]
            rmfLiftStateMsg.lift_time = self.get_clock().now().to_msg()
            self.lift_state_pub.publish(rmfLiftStateMsg)


    def liftRequestCallBack(self, msg):

        # Check if Request Liftname is in this Kone Lift-List Setup
        if not msg.lift_name in self.koneAdaptorGalen.liftNameList:
            self.get_logger().info("Request not for these list of Kone lifts. Dropped!")
            return

        # End session
        if msg.request_type == 0:
            self.get_logger().info("Recieved an end session. Ending session")
            self.koneAdaptorGalen.updateSessionID(msg.lift_name, "")
            return

        # Update session ID
        self.koneAdaptorGalen.updateSessionID(msg.lift_name, msg.session_id) 


        current_lift_index = self.koneAdaptorGalen.liftNameList.index(msg.lift_name)

        # prev req == cur req and time elapsed < 60 seconds, then return and do nothing
        if (
            self.prev_rmf_lift_request[current_lift_index].destination_floor == msg.destination_floor
            and self.prev_rmf_lift_request[current_lift_index].lift_name == msg.lift_name
            and self.prev_rmf_lift_request[current_lift_index].session_id == msg.session_id
        ):
            req_time_elapsed = msg.request_time.sec - self.prev_rmf_lift_request[current_lift_index].request_time.sec
            print ("req_time_elapsed: " + str(req_time_elapsed))
            if (req_time_elapsed < 60):
                self.get_logger().info("Duplicated lift request. Skipping !")
                return


        # Thsi is a valid lift request, update the request info now
        current_dest_floor = msg.destination_floor
        current_source_floor = self.koneAdaptorGalen.current_liftstate_list[current_lift_index].current_floor

        if current_dest_floor != current_source_floor :

            self.koneAdaptorGalen.updateDestFloor(msg.lift_name, msg.destination_floor)

            self.get_logger().info(
                "Sending lift command paylod now. Lift: %s, Destination Floor: %s" %(msg.lift_name, msg.destination_floor)
            )
            
            self.koneAdaptorGalen.liftDestinationCall(current_source_floor, current_dest_floor, msg.lift_name)
            # self.koneAdaptorGalen.liftLandingCall(current_dest_floor, msg.lift_name)

            # Update prev_rmf_lift_request to latest request
            self.prev_rmf_lift_request[current_lift_index] = msg
        else:
            self.get_logger().info("Lift: %s , Destination floor == Current floor. Skipping!" % msg.lift_name ) 



def main(args=None):
    rclpy.init(args=args)
    try:
        lift_node = LiftNode("Kone")
        rclpy.spin(lift_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        lift_node.destroy_node()


if __name__ == "__main__":
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()
