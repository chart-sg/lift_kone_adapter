#!/usr/bin/python
import requests, json
import pprint
import websocket, websockets
from websocket import create_connection
from websockets import connect
from requests.sessions import ChunkedEncodingError, session
import time

# from rmf_lift_msgs.msg import LiftState, LiftRequest

import yaml
import logging
from typing import Dict, List, Tuple

import rclpy
import rclpy.node

import os
from ament_index_python.packages import get_package_share_directory


class koneAdaptor:
    def __init__(
        self, access_id: str, access_secret: str, liftName: str, logger: logging
    ):

        self.token_response = requests.post(
            self.auth_server_url,
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.client_id, self.client_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        print(responseInDict)
        self.loadSessionParameters(responseInDict)
        # print(self.buildingID)
        self.logger.info("Session parameters loaded")
        self.connectionURL = (
            "wss://dev.kone.com/stream-v1?accessToken=" + self.sessionToken
        )
        self.getBuildingTopo(self.buildingID)
        # return self.sessionToken

    def printBuildingDetails(self):
        self.getToken()
        self.logger.info("Building ID is:%s" % self.buildingID)
        self.logger.info("Lift group is:%s" % self.liftGroup)
        self.logger.info("Lift IDs are:%s" % self.liftIDList)
        self.logger.info("Lift names are:%s" % self.liftNameList)
        self.logger.info("Available floors are: %s" % self.listOfAvailableFloors)

    def getToken(self):
        self.token_response = requests.post(
            self.auth_server_url,
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.client_id, self.client_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        print(responseInDict)
        self.loadSessionParameters(responseInDict)
        # print(self.buildingID)
        self.logger.info("Session parameters loaded")
        self.connectionURL = (
            "wss://dev.kone.com/stream-v1?accessToken=" + self.sessionToken
        )
        self.getBuildingTopo(self.buildingID)
        return self.sessionToken


class koneAdaptor(rclpy.node.Node):
    def __init__(self):
        super().__init__("minimal_param_node")
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.parsed_yaml = self.get_config()

        self.token_response = requests.post(
            self.parsed_yaml["auth_server_url"],
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.access_id, self.access_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        print(responseInDict)
        self.loadSessionParameters(responseInDict)
        # print(self.buildingID)
        self.logger.info("Session parameters loaded")
        # self.connectionURL = (
        #     "wss://dev.kone.com/stream-v1?accessToken=" + self.sessionToken
        # )

    def timer_callback(self):
        self.get_logger().info("Running")  #% my_param)

    def get_config(self):
        config = os.path.join(
            get_package_share_directory("kone_ros_api"), "config", "env.yaml"
        )
        with open(config, "r") as stream:
            try:
                parsed_yaml = yaml.safe_load(stream)
                print(parsed_yaml)
                print("Succcess reading env.yaml")
            except yaml.YAMLError as exc:
                print(exc)
                print("Failure reading env.yaml")
            return parsed_yaml


def main():
    rclpy.init()
    node = koneAdaptor()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
