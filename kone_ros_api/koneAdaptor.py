#!/usr/bin/python
from pprint import pprint
import requests, json

# from rmf_lift_msgs.msg import LiftState, LiftRequest

import yaml
from typing import Dict, List, Tuple

import rclpy
import rclpy.node

import os
from ament_index_python.packages import get_package_share_directory

import websocket


class koneAdaptor(rclpy.node.Node):
    def __init__(self):
        super().__init__("kone_adaptor_node")
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.parsed_yaml = self.get_config()
        self.token_response = requests.post(
            url=self.parsed_yaml["auth_server_url"],
            data=self.parsed_yaml["token_req_payload"],
            allow_redirects=False,
            headers=self.parsed_yaml["requestHeaders"],
            auth=(self.parsed_yaml["access_id"], self.parsed_yaml["access_secret"]),
        )
        # print(type(self.token_response))
        if self.token_response.status_code != 200:
            print(
                "Failed to get access token from Kone API server. Is there internet service?"
            )
            return
        print("Got access token from Kone API server")
        self.access_key = json.loads(self.token_response.content)
        # pprint(access_key)

        # Serialize base json message

        payload = {
            "type": "site-monitoring",
            "requestId": "1234",
            "buildingId": "building:4TFxWRCv23D",
            "callType": "monitor",
            "groupId": "1",
            "payload": {
                "sub": "smartOfficeLiftSouthWing_user123",  # Subscriber has to re-activate the topic before this time runs out.
                "duration": 300,
                "subtopics": ["lift_1/position"],
            },
        }
        self.get_logger().info(
            "sending lift command with the following payload...%s" % payload
        )
        self.ws = websocket.WebSocketApp(
            url=self.parsed_yaml["ws_url"] + self.access_key["access_token"],
            subprotocols=["koneapi"],
            on_message=lambda ws, msg: self.onSocketMsg(msg),
            on_error=lambda ws, msg: self.onSocketError(msg),
            on_open=lambda ws: self.sendCommandviaSocket(payload),
            on_close=lambda ws, closeCode, closeMsg: self.closeSocketMsg(
                closeCode, closeMsg
            ),
        )
        self.ws.run_forever()

    def timer_callback(self):

        self.get_logger().info("Running")  #% my_param)

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

    def closeSocketMsg(self, closeCode, closeMsg):
        self.get_logger().info("Websocket closing with msg %s" % closeMsg)
        self.get_logger().info(
            "###################### WEBSOCKERCLOSING ########################"
        )
        # self.get_logger().info("Websocket closed session with CloseCode: %i"% closeCode)

    def onSocketError(self, msg):
        # self.listOfMessage.append(msg)
        self.get_logger().info("%s" % msg)

    def sendCommandviaSocket(self, payload):
        self.get_logger().info("doing sending tasks")
        self.ws.send(json.dumps(payload))

    def onSocketMsg(self, message):
        msg = json.loads(message)
        self.get_logger().info("received message is %s" % msg)
        # typeOfMsg = msg["type"]
        # self.listOfMessage.append(msg)
        # if typeOfMsg == "lift-door-state":
        #     self.updateDoorStatus(msg)
        #     self.updateMotionState(msg)
        #     self.updateCurrentFloor(msg)
        #     self.updateLiftName(msg)
        #     self.get_logger().info("Lift ID is %s" % (self.liftName))
        #     self.get_logger().info(
        #         "Current level %s and door status is %s."
        #         % (str(int(self.currentFloor)), self.doorState)
        #     )
        #     self.get_logger().info("Current lift motion is %s" % self.motionState)


def make_elevator_call():
    pass


def hold_car_door_open():
    pass


def cancel_elevator_call():
    pass


def site_monitoring():
    payload = {
        "type": "site-monitoring",
        "requestId": "01841d1c-f4ba-4f9c-a348-6f679bfae86e",
        "buildingId": "targetBuildingId",
        "callType": "monitor",
        "groupId": "1",
        "payload": {
            "sub": "smartOfficeLiftSouthWing_user123",
            "duration": 300,
            "subtopics": ["call_state/123/fixed"],
        },
    }

    pass


def common_commands():
    pass


def monitoring_events():
    pass


def main():
    rclpy.init()
    node = koneAdaptor()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
