#!/usr/bin/env python3
# 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

from numpy import empty
import rclpy
import sys
from rclpy.node import Node
from koneAdaptor.koneAdaptor import koneAdaptor
from rmf_lift_msgs.msg import LiftState, LiftRequest

import time
from threading import Thread


class LiftNode(Node):
    def __init__(self, lift_name):
        self.node_name = f"{lift_name}_lift_node"
        super().__init__(self.node_name)

        # SANDBOX DETAILS
        # self.clientID = 'd5b230bc-f4ce-4ba7-98a2-0dbff56023ee'
        # self.clientSecret = 'e1a4f05c36aad0c0df4877314c5e760f4b2bf6d98bcf4007aefbb6128684dded'

        # ACTUAL DETAILS
        self.clientID = "738ae962-90e0-40a1-9113-09a031664301"
        self.clientSecret = (
            "eac75d54b4f5c638fdcd858a784e0591fe78a14f54355fde431932745d9a1d83"
        )
        self.rmfSessionID = ""
        self.sesh = 0
        self.latch = False

        # Var for storing previous session
        self.prev_rmf_lift_request = LiftRequest
        # self.current_lift_destination = LiftRequest.destination_floor

        self.koneAdaptorA = koneAdaptor(
            self.clientID, self.clientSecret, "A", self.get_logger()
        )
        # self.koneAdaptorB = koneAdaptor(self.clientID, self.clientSecret, 'B', self.get_logger())

        self.lift_state_pub = self.create_publisher(LiftState, "lift_states", 10)

        self.lift_request_sub = self.create_subscription(
            LiftRequest, "lift_requests", self.liftRequestCallBack, 10
        )

        self.get_logger().info("Lift subscriber is up with /lift_requests topic")

        threadA = Thread(target=self.liftStatePublisherCallBack, name="A", args=("A",))
        threadA.daemon = True
        threadA.start()
        self.get_logger().info("Lift publisher is up with /lift_state topic!")

    def liftStatePublisherCallBack(self, liftName):
        while True:
            if self.sesh == 0:
                liftStatus = LiftState()
                liftStatus.lift_name = "B"
                liftStatus.current_floor = "Empty Init"
                liftStatus.destination_floor = "L6"
                liftStatus.door_state = 0
                liftStatus.motion_state = 0
                liftStatus.current_mode = 2
                liftStatus.available_modes = [0, 1, 2, 3, 4, 5]
                liftStatus.available_floors = ["L0", "L1", "L2", "L3", "L4", "L5", "L6"]
                liftStatus.lift_time = self.get_clock().now().to_msg()
                liftStatus.session_id = ""
                self.lift_state_pub.publish(liftStatus)

                self.get_logger().info(
                    "###################### PROCEEDING TO SESH 1 ########################"
                )
                self.sesh = 1

            elif self.sesh == 1:  # Dummy call to see which one we get
                self.get_logger().info("Recieved Initial lift Landing request")

                payload = {
                    "type": "lift-call",
                    "callType": "robot",
                    "callAction": "landing",
                    "buildingId": "building:PZFtaS27eW",
                    "sourceId": "area:PZFtaS27eW:2000",
                    "direction": "up",
                    "monitorEvents": ["door"],
                    "keepAlive": "false",
                }

                self.koneAdaptorA.sendLiftCommand(payload)
                self.koneAdaptorA.runSocketTilComplete()

                if (
                    self.koneAdaptorA.liftName == "A"
                ):  # If you got A then you actually got B
                    self.get_logger().info("Got the wrong lift, sending it away")
                    sourceDestination = "area:PZFtaS27eW:" + "2" + "000"
                    payloadDestination = "area:PZFtaS27eW:" + "7" + "000"

                    payload = {
                        "type": "lift-call",
                        "callType": "robot",
                        "callAction": "destination",
                        "buildingId": "building:PZFtaS27eW",
                        "sourceId": sourceDestination,  # "area:PZFtaS27eW:1000",
                        "destinationId": payloadDestination,
                        # "direction": "up",
                        # "monitorEvents": ["call"],
                        # "monitorEvents": ["deck"],
                        "monitorEvents": ["door"],
                        "keepAlive": "false",
                    }
                    self.koneAdaptorA.sendLiftCommand(payload)
                    self.koneAdaptorA.runSocketTilComplete()

                elif self.koneAdaptorA.liftName == "B":
                    self.get_logger().info("Got the CORRECT lift YAY")
                    self.get_logger().info(
                        "###################### PROCEEDING TO SESH 2 ########################"
                    )
                    self.sesh = 2

            elif self.sesh == 2:

                rmfLiftStateMsg = self.koneAdaptorA.getStatus()

                if self.koneAdaptorA.liftName == "A":
                    self.sesh = 1
                    return

                if not rmfLiftStateMsg.destination_floor:
                    self.get_logger().info(
                        "Lift state with empty destination. Dropping"
                    )
                    return

                if self.koneAdaptorA.currentFloor == 1 and self.latch == False:
                    rmfLiftStateMsg.session_id = self.rmfSessionID
                    rmfLiftStateMsg.lift_time = self.get_clock().now().to_msg()
                    rmfLiftStateMsg.door_state = 0
                    self.lift_state_pub.publish(rmfLiftStateMsg)
                    self.latch = True
                    time.sleep(5)
                else:
                    rmfLiftStateMsg.session_id = self.rmfSessionID
                    rmfLiftStateMsg.lift_time = self.get_clock().now().to_msg()
                    self.lift_state_pub.publish(rmfLiftStateMsg)
                    time.sleep(5)

    def liftRequestCallBack(self, msg):

        if self.sesh == 2:
            payload = {
                "type": "lift-call",
                "callType": "robot",
                "callAction": "destination",
                "buildingId": "building:PZFtaS27eW",
                "sourceId": "area:PZFtaS27eW:2000",
                "destinationId": "area:PZFtaS27eW:7000",
                "monitorEvents": ["door"],
                "keepAlive": "false",
            }
            self.koneAdaptorA.sendLiftCommand(payload)
            self.koneAdaptorA.runSocketTilComplete()
            self.get_logger().info(
                "###################### PROCEEDING TO SESH 3 ########################"
            )
            self.sesh = 3
        elif self.sesh == 3:
            # Pack into right payload msg!
            self.get_logger().info("Recieved lift request")
            rmfLiftRequestMsg = msg

            if not msg.lift_name:
                self.get_logger().info("Empty lift name. Dropped!")
                return

            if self.prev_rmf_lift_request.destination_floor == msg.destination_floor:
                self.get_logger().info(
                    "Lift Request's Destination Floor == Current Lift Destination Floor. Skipping."
                )
                return

            if (
                self.prev_rmf_lift_request.destination_floor == msg.destination_floor
                and self.prev_rmf_lift_request.lift_name == msg.lift_name
                and self.prev_rmf_lift_request.session_id == msg.session_id
            ):
                self.get_logger().info("Duplicated lift request. Skipping !")
                return

            self.prev_rmf_lift_request = msg

            self.get_logger().info(
                "Proceeding with lift request! with dest floor %s"
                % (self.prev_rmf_lift_request.destination_floor)
            )

            dest_floor = rmfLiftRequestMsg.destination_floor[-1]
            if rmfLiftRequestMsg.request_type == 0:
                self.rmfSessionID = ""
            else:
                self.rmfSessionID = rmfLiftRequestMsg.session_id

            self.koneAdaptorA.updateDestFloor(rmfLiftRequestMsg)

            self.get_logger().info(
                "Previous lift request msg %s %s %s"
                % (
                    self.prev_rmf_lift_request.destination_floor,
                    self.prev_rmf_lift_request.lift_name,
                    self.prev_rmf_lift_request.session_id,
                )
            )
            self.get_logger().info(
                "Current lift request Callback Destination. Lift %s Floor %s"
                % (msg.lift_name, msg.destination_floor)
            )

            dest_floor = str(int(dest_floor) + 1)

            payload = {
                "type": "lift-call",
                "callType": "robot",
                "callAction": "destination",
                "buildingId": "building:PZFtaS27eW",
                "sourceId": "area:PZFtaS27eW:2000",
                "destinationId": "area:PZFtaS27eW:" + dest_floor + "000",
                "monitorEvents": ["door"],
                "keepAlive": "false",
            }
            self.koneAdaptorA.sendLiftCommand(payload)
            self.koneAdaptorA.runSocketTilComplete()
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    lift_name = "B"
    lift_node = LiftNode(lift_name)
    lift_node.get_logger().info("Kone lift node created")

    try:
        rclpy.spin(lift_node)

    finally:
        lift_node.destroy_node()
        rclpy.shutdown()


# TESTING COMMANDS
# ros2 topic pub /door_requests rmf_door_msgs/msg/DoorRequest "{request_time: {sec: 0, nanosec: 0},requester_id: '',door_name: 'Dorma',requested_mode:{ value: 2}}" --once
"""
ros2 topic pub -1 /lift_request rmf_msgs/LiftRequest "{
    lift_name: 'kone', 
    session_id: 'some_session_id_1', 
    request_type: 1, 
    destination_floor: '5', 
    door_state: 2}"
"""
