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


class koneAdaptor(rclpy.node.Node):
    def __init__(self):
        super().__init__("minimal_param_node")
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
        responseInDict = json.loads(self.token_response.content)
        pprint(responseInDict)

    def timer_callback(self):
        self.get_logger().info("Running")  #% my_param)

    def get_config(self):
        config = os.path.join(
            get_package_share_directory("kone_ros_api"), "config", "env.yaml"
        )
        with open(config, "r") as stream:
            try:
                parsed_yaml = yaml.safe_load(stream)
                # print(parsed_yaml)
                self.get_logger().info("Succcess reading env.yaml")
            except yaml.YAMLError as exc:
                # print(exc)
                self.get_logger().info("Failure reading env.yaml")
            return parsed_yaml


def make_elevator_call():
    pass


def hold_car_door_open():
    pass


def cancel_elevator_call():
    pass


def site_monitoring():
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
