#!/usr/bin/python
import requests
import json
import pprint
import websocket
from websocket import create_connection

# from websockets import connect
from requests.sessions import ChunkedEncodingError, session
import time
from rmf_lift_msgs.msg import LiftState, LiftRequest

from pprint import pprint
import datetime
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import rclpy
import rclpy.node

import yaml

class koneAdaptor:
    def __init__(self, config_yaml : yaml):
       
        ######################################
        #######--Parameter Setting---#########
        ######################################

        self.baseURL = config_yaml['baseURL']
        self.auth_server_url =  config_yaml['auth_server_url']
        self.requestHeaders = config_yaml['requestHeaders']
        self.subproto = ["koneapi"]

        # galen lift
        # self.token_req_payload = {"grant_type": "client_credentials", "scope": "robotcall/group:PZFtaS27eW:1 application/inventory", } # PZFtaS27eW is the building id for Galen

        # sandbox lift
        self.token_req_payload = config_yaml['token_req_payload']
        

        #####################################
        #####################################


        self.connectionURL = str
        self.client_id = config_yaml['access_id']
        self.client_secret = config_yaml['access_secret']
        self.buildingID = config_yaml['buildingId']
        self.door_holding_duration = config_yaml['liftdoor_holding_duration']
        self.token_response = None
        # note the square bracket for WebSocketApp purposes
        
        self.sessionToken = str
        self.sessionTokenType = None
        self.scopeOfAccess = None

        self.liftID = None
        self.liftIDList = []
        self.liftstate_monitoring_topic_list= []
        self.liftNameList = []
        self.liftGroup = None
        self.floorAreaList = []
        self.floorNameList = []
        self.liftDeckList = []
        self.areaLevelDict = {}
        self.liftnameliftDeckDict = {}
        self.BuildingAvailableFloor = []
        self.liftTerminalList = []


        self.ws = None
        self.ws_state = None
        self.ws_config = None
        self.listOfMessage = []
        
        self.current_liftstate_list = []
        self.last_active_timestamp_for_liftstate_ws = 0
        


        print("KoneAdaptor V2 is alive!")
        self.getToken()
        self.getLiftConfig() # getting buildingTopo for api v2
        self.initCurrentLiftstateList()


    # def get_config(self):
    #     config = os.path.join(
    #         get_package_share_directory("koneAdaptor"), "config", "env.yaml"
    #     )
    #     with open(config, "r") as stream:
    #         try:
    #             parsed_yaml = yaml.safe_load(stream)
    #             # print(parsed_yaml)
    #             self.get_logger().info("Succcess reading env.yaml")
    #         except yaml.YAMLError as exc:
    #             # print(exc)
    #             self.get_logger().info("Failure reading env.yaml")
    #         return parsed_yaml


    def initCurrentLiftstateList(self):
        for i in range (len(self.liftNameList)):
            self.current_liftstate_list.append(LiftState())
            self.current_liftstate_list[i].lift_name = self.liftNameList[i]
            self.current_liftstate_list[i].available_modes =  [0, 1, 2, 3, 4, 5]
            self.current_liftstate_list[i].current_mode = 2
            self.current_liftstate_list[i].available_floors = self.BuildingAvailableFloor
            self.current_liftstate_list[i].current_floor = "L1"
            self.current_liftstate_list[i].door_state = 0
            self.current_liftstate_list[i].motion_state = 0

    def getToken(self):
        self.token_response = requests.post(
            self.auth_server_url,
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.client_id, self.client_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        #self.logger('Initial response from websocket is \n')
        #self.logger(responseInDict)
        # print ('\nInitial response from websocket is \n')
        # pprint (responseInDict)

        self.sessionToken = responseInDict['access_token']
        self.sessionTokenType = responseInDict['token_type']

        print("\nAccess Token & Scope are loaded.")
        self.connectionURL = (
            "wss://dev.kone.com/stream-v2?accessToken=" + self.sessionToken
        )


    def getResource(self):
        self.token_response = requests.post(
            self.auth_server_url,
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.client_id, self.client_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        self.sessionToken = responseInDict['access_token']
        self.scopeOfAccess = responseInDict['scope']
        print ("\naccess_token: ")
        pprint(self.sessionToken)
        print ("\nscope: ")
        pprint(self.scopeOfAccess)

        getResourceURL = self.baseURL + '/api/v2/application/self/resources'
        #print (getResourceURL)
        bearer = "Bearer" + " " + self.sessionToken
        resourceRequestHeader = {'Authorization': bearer}
        resourceResponse = requests.get(getResourceURL, headers=resourceRequestHeader)
        responseinDict = json.loads(resourceResponse.content)
        
        print('\n')
        print("HERE ARE THE RAW RESOURCE DATA: \n")
        pprint(responseinDict)
        print('\n')

    def getUserInfo(self):
        self.token_response = requests.post(
            self.auth_server_url,
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.client_id, self.client_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        self.sessionToken = responseInDict['access_token']
        self.scopeOfAccess = responseInDict['scope']
        print ("\naccess_token: ")
        pprint(self.sessionToken)
        print ("\nscope: ")
        pprint(self.scopeOfAccess)

        getResourceURL = self.baseURL + '/api/v2/oauth2/userinfo'
        #print (getResourceURL)
        bearer = "Bearer" + " " + self.sessionToken
        resourceRequestHeader = {'Authorization': bearer}
        resourceResponse = requests.get(getResourceURL, headers=resourceRequestHeader)
        responseinDict = json.loads(resourceResponse.content)
        
        print('\n')
        print("HERE ARE THE RAW USER INFO DATA: \n")
        pprint(responseinDict)
        print('\n')

    def getBuildingConfig(self):
        self.token_response = requests.post(
            self.auth_server_url,
            data=self.token_req_payload,
            allow_redirects=False,
            headers=self.requestHeaders,
            auth=(self.client_id, self.client_secret),
        )
        responseInDict = json.loads(self.token_response.content)
        self.sessionToken = responseInDict['access_token']
        self.scopeOfAccess = responseInDict['scope']
        print ("\naccess_token: ")
        pprint(self.sessionToken)
        print ("\nscope: ")
        pprint(self.scopeOfAccess)

        payload = {"type": "common-api",
            "requestId": "1",
            "buildingId": "building:PZFtaS27eW",
            "callType": "config",
            "groupId": "1",
            }

        self.sendLiftCommand(payload)
        self.runSocketTilComplete()

    def sendLiftCommand(self, payload):
        print("sending lift command with the following payload %s" % payload)
        self.ws = websocket.WebSocketApp(
            url=self.connectionURL,
            subprotocols=self.subproto,
            on_message=lambda ws, msg: self.onSocketMsg(msg),
            on_error=lambda ws, msg: self.onSocketError(msg),
            on_open=lambda ws: self.sendCommandviaSocket(payload),
            on_close=lambda ws, closeCode, closeMsg: self.closeSocketMsg(closeCode),
        )

    def openLiftStateWS(self):
        print ("\nOpen liftstate websocket.")
        self.last_active_timestamp_for_liftstate_ws = time.time()

        payload = {"type": "site-monitoring",
            "requestId": "1",
            "buildingId": self.buildingID,
            "callType": "monitor",
            "groupId": "1",
            "payload": {"sub": "get_lift_state", 
                        "duration": 300, 
                        "subtopics": self.liftstate_monitoring_topic_list
                        },
            }

        self.ws_state = websocket.WebSocketApp(
            url=self.connectionURL,
            subprotocols=self.subproto,
            on_message=lambda ws_state, msg: self.onSocketMsg_liftstate(msg),
            on_error=lambda ws_state, msg: self.onSocketError(msg),
            on_open=lambda ws_state: self.sendCommandviaSocket_state(payload),
            on_close=lambda ws_state, closeCode, closeMsg: self.closeSocketMsg_state(closeCode),
        )

    def closeSocketMsg_state(self, closeCode):
        print("###################### WEBSOCKERCLOSING ########################")
        print("Websocket liftstate closed session with CloseCode: ", closeCode)
        self.ws_state.close()

    def closeSocketMsg(self, closeCode):
        # print("###################### WEBSOCKERCLOSING ########################")
        # print("Websocket closed session with CloseCode: ", closeCode)
        self.ws.close()
    
    def closeSocketMsg_config(self, closeCode):
        # print("Websocket liftconfig closed session with CloseCode: ", closeCode)
        self.ws_config.close()

    def onSocketMsg_liftstate(self, message):
        self.last_active_timestamp_for_liftstate_ws = time.time()

        msg = json.loads(message)
        # print("\nreceived liftstate message: ", message)
        try:
            typeOfMsg = msg["subtopic"]
        except:
            typeOfMsg = msg["statusCode"]
        if typeOfMsg == 201:
            print ("\nLiftstate monitoring event in progress...\n")
        elif typeOfMsg in self.liftstate_monitoring_topic_list:
            self.updateLiftStateList(msg)

    def onSocketMsg(self, message):
        msg = json.loads(message)
        print("\nreceived message: ")
        pprint(msg)
        try:
            typeOfMsg = msg["statusCode"]
        except:
            typeOfMsg = msg["data"]["success"]
        if typeOfMsg == True:
            print ("Sent lift command successfully.")
        elif typeOfMsg == 201:
            print ("Received lift command ack.")
        else:
            self.closeSocketMsg(0)

    def decodeLiftConfigMsg(self, msg):
        lift_group_selected = 0 # to get the first lift group

        try:
            responseinDict = msg["data"]
            self.liftGroup = responseinDict["groups"][lift_group_selected]["group_id"]
        except:
            print ("Error in getting group with group index: " + str(lift_group_selected))

        # getting lift terminal info
        try:
              self.liftTerminalList = responseinDict["groups"][lift_group_selected]["terminals"]
        except:
            print ("Error in getting lifts terminal list.")

        # getting lifts name, lift id, lift deck
        try:
            for lift in responseinDict["groups"][lift_group_selected]["lifts"]:
                self.liftNameList.append(lift["lift_name"])
                self.liftIDList.append(lift["lift_id"])
                self.liftDeckList.append(lift["decks"][0]["area_id"])
                self.liftnameliftDeckDict[lift["lift_name"]] = lift["decks"][0]["area_id"]
        except:
            print ("Error in getting lifts name, lift id, lift deck.")


        # getting area id 
        try:
            for dest in responseinDict["destinations"]:
                self.floorAreaList.append(dest["area_id"])
                self.floorNameList.append(dest["short_name"])
                floor_name_from_config = ""
                if dest["short_name"][0] == "B":
                    floor_name_from_config = dest["short_name"]
                else:
                    floor_name_from_config = "L" + dest["short_name"]
                self.areaLevelDict[dest["area_id"]] = floor_name_from_config
                self.BuildingAvailableFloor.append(floor_name_from_config)
        except:
            print ("Error in getting floorAreaList, floorNameList.")

        # generate liftstate monitoring subtopic based on liftid feedback from building topo
        try:
            for i in range (len(self.liftIDList)):
                topic = "lift_" + str(self.liftIDList[i]) + "/doors"
                self.liftstate_monitoring_topic_list.append(topic)
        except:
            print ("Error in generating liftstate monitoring subtopic.")
            self.liftstate_monitoring_topic_list = ["lift_1/doors", "lift_2/doors", "lift_3/doors"]


        print ("\nGetting lift config:")
        print ("Building ID: " + self.buildingID)
        print ("Lift group: " + str(self.liftGroup))
        print ("Lift name: " + str(self.liftNameList)) 
        print ("Lift ID: " + str(self.liftIDList))
        print ("Lift deck name: " + str(self.liftDeckList))
        print ("Floor Area: " + str(self.floorAreaList))
        print ("Floor name: " + str(self.floorNameList))
        print ("areaLevelDict: " + str(self.areaLevelDict))
        print ("liftnameliftDeckDict: " + str(self.liftnameliftDeckDict))
        print ("Liftstate monitoring Topic List: " + str(self.liftstate_monitoring_topic_list))
        print ("Lift Terminal List: " + str(self.liftTerminalList))

        print('\n')


    def onSocketMsg_liftconfig(self, message):
        msg = json.loads(message)
        # print("\nreceived liftconfig message: ", message)
        try:
            typeOfMsg = msg["statusCode"]
        except:
            typeOfMsg = msg["callType"]
        if typeOfMsg == "config":
            # pprint (msg)
            self.decodeLiftConfigMsg(msg)
            self.closeSocketMsg_config(0)
        elif typeOfMsg == 201:
            print ("Received lift config ack.")
            

    def openLiftConfigWS(self):
        print ("\nOpen liftconfig websocket.")

        payload = {"type": "common-api",
            "requestId": "1",
            "buildingId": self.buildingID,
            "callType": "config",
            "groupId": "1",
            }

        self.ws_config = websocket.WebSocketApp(
            url=self.connectionURL,
            subprotocols=self.subproto,
            on_message=lambda ws_config, msg: self.onSocketMsg_liftconfig(msg),
            on_error=lambda ws_config, msg: self.onSocketError(msg),
            on_open=lambda ws_config: self.sendCommandviaSocket_config(payload),
            on_close=lambda ws_config, closeCode, closeMsg: self.closeSocketMsg_config(closeCode),
        )

    def sendCommandviaSocket(self, payload):
        print("Payload sent: ", payload)
        self.ws.send(json.dumps(payload))
    
    def sendCommandviaSocket_state(self, payload):
        print("Payload sent: ", payload)
        self.ws_state.send(json.dumps(payload))

    def sendCommandviaSocket_config(self, payload):
        print("Payload sent: ", payload)
        self.ws_config.send(json.dumps(payload))

    def onSocketError(self, msg):
        self.listOfMessage.append(msg)
        pprint(msg)

    def runSocketTilComplete(self):
        self.ws.run_forever()
        return False

    def runSocketTilComplete_config(self):
        self.ws_config.run_forever()
        return False

    def runSocketTilComplete_state(self):
        self.ws_state.run_forever()
        return False

    def updateLiftStateList(self, msg):
        # only update floor, door state here

        cur_liftname = 0
        cur_floor = "L1"
        cur_doorstate = 0
        try:
            msg_content = msg["subtopic"].split("/")
            if msg_content[1] == "doors":
                lift_index = msg_content[0].split("_")
                if lift_index[0] == "lift":
                    cur_liftname = lift_index[1]
                    cur_floor = self.getCurrentFloor(msg)
                    cur_doorstate = self.getDoorState(msg)
                    
        except:
            print ("Failed in decoding liftstate data from websocket.")
            return
        
        # Holding door opening time
        if (cur_doorstate in [1,2]):    # door state = OPENING/OPENED
            # self.liftDoorHoldingCall(str(cur_floor), self.current_liftstate_list[int(cur_liftname)-1].lift_name, self.door_holding_duration)
            print("NOT Holding lift " + self.current_liftstate_list[int(cur_liftname)-1].lift_name + " door at " + str(cur_floor))

        self.current_liftstate_list[int(cur_liftname)-1].current_floor = str(cur_floor)
        self.current_liftstate_list[int(cur_liftname)-1].door_state = cur_doorstate

        print ("Updated LiftState lift: " + self.current_liftstate_list[int(cur_liftname)-1].lift_name + ", floor: " + self.current_liftstate_list[int(cur_liftname)-1].current_floor + ", door: " + str(self.current_liftstate_list[int(cur_liftname)-1].door_state)+ ","+ msg["data"]["state"])


    def getCurrentFloor(self, msg):
        current_floor = "L1"
        areaID = msg["data"]["landing"]
        try:
            current_floor = self.areaLevelDict[areaID]
        except:
            current_floor = "L1"
        return current_floor

    def getDoorState(self, msg):
        doorState = 0
        raw_doorState = msg["data"]["state"]
        if raw_doorState == "OPENED":
            doorState = 2
        elif raw_doorState == "CLOSED":
            doorState = 0
        elif raw_doorState == "OPENING":
            doorState = 1
        elif raw_doorState == "CLOSING":
            doorState = 0   # put 0 here because sometime will receive a CLOSING even after door CLOSED from websocket 
        return doorState

    def updateDestFloor(self, liftname, newDestFloor):
        lift_index = self.liftNameList.index(liftname)
        self.current_liftstate_list[lift_index].destination_floor = newDestFloor

    def updateSessionID(self,liftname,newSessionID):
        lift_index = self.liftNameList.index(liftname)
        self.current_liftstate_list[lift_index].session_id = newSessionID

    def generatePayload_LiftDestinationCall(self, sourceLvl, destLvl, liftname):
        lift_selected = self.liftnameliftDeckDict[liftname]
        current_source_floor_areaID = dict((v,k) for k,v in self.areaLevelDict.items()).get(sourceLvl)
        current_dest_areaID = str(dict((v,k) for k,v in self.areaLevelDict.items()).get(destLvl))
        payload = {
            "type": "lift-call-api-v2",
            "buildingId": self.buildingID,
            "callType": "action",
            "groupId": "1",
            "payload": {
                "request_id": 1,
                "area": current_source_floor_areaID,
                "time": datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat(),
                "terminal": self.liftTerminalList[0],
                "call": { 
                    "action": 2, 
                    "destination": current_dest_areaID,
                    "allowed_lifts": [lift_selected]
                    }
                }
            }
        return payload

    def liftDestinationCall(self, sourceLvl, destLvl, liftname):
        payload = self.generatePayload_LiftDestinationCall(sourceLvl, destLvl, liftname)
        self.sendLiftCommand(payload)
        self.runSocketTilComplete()

    def generatePayload_DoorHolding(self, floor, liftname, holding_duration):
        lift_selected = self.liftnameliftDeckDict[liftname]
        floor_areaID = str(dict((v,k) for k,v in self.areaLevelDict.items()).get(floor))
        print ("Door holding lift: " + str(lift_selected) + "; floor: " + floor_areaID)
        payload ={
                    "type": "lift-call-api-v2",
                    "buildingId": self.buildingID,
                    "callType": "hold_open",
                    "groupId": '1',
                    "payload": {
                        "time": datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat(),
                        "lift_deck": lift_selected,
                        "served_area": floor_areaID,
                        "hard_time": holding_duration,
                        "soft_time": 20,
                    },
                }
        return payload
    
    def liftDoorHoldingCall(self, floor, liftname, holding_duration):
        payload = self.generatePayload_DoorHolding(floor, liftname, holding_duration)
        self.sendLiftCommand(payload)
        self.runSocketTilComplete()

    def generatePayload_LiftLandingCall(self, sourceLvl, liftname):
        lift_selected = self.liftnameliftDeckDict[liftname]
        source_floor_areaID = str(dict((v,k) for k,v in self.areaLevelDict.items()).get(sourceLvl))

        payload = {
            "type": "lift-call-api-v2",
            "buildingId": self.buildingID,
            "callType": "action",
            "groupId": "1",
            "payload": {
                "request_id": 1,
                "area": source_floor_areaID,
                "time": datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat(),
                "terminal": self.liftTerminalList[0],
                "call": { 
                    "action": 2, 
                    "allowed_lifts": [lift_selected]
                    }
                }
            }
        return payload

    def liftLandingCall(self, sourceLvl, liftname):
        payload = self.generatePayload_LiftLandingCall(sourceLvl, liftname)
        self.sendLiftCommand(payload)
        self.runSocketTilComplete()
    
    def getLiftConfig(self):
        self.openLiftConfigWS()
        self.runSocketTilComplete_config()


def main():
    #ONLY USED FOR ISOLATED TESTING
    #TEMP ID AND SECRET ALSO WORKS FOR HEARTBEAT
    
    #sandbox
    clientID = '160fd5a3-2010-4348-8aec-644bb100bb93'
    clientSecret = '592a91ee0c89005933d6cef74ff0bb66cd9953d5f19c4e57d4ea6b6e7810c4c3'
    buildingId= "building:HxKjGc3knnh"
    
    galenAdaptor = koneAdaptor(clientID, clientSecret, buildingId)
    # galenAdaptor.getResource()
    # galenAdaptor.getUserInfo()
    # galenAdaptor.getBuildingConfig()

    #real lift
    # payload = {"type": "site-monitoring",
    #             "requestId": "1",
    #             "buildingId": "building:PZFtaS27eW",
    #             "callType": "monitor",
    #             "groupId": "1",
    #             "payload": {"sub": "test", 
    #                         "duration": 300, 
    #                         "subtopics": ['lift_1/position']
    #                         },
    #             }

    #sandbox lift call
    # payload = {
    #     "type": "lift-call-api-v2",
    #     "buildingId": "building:HxKjGc3knnh",
    #     "callType": "action",
    #     "groupId": "1",
    #     "payload": {
    #         "request_id": 1,
    #         "area": 3000,
    #         "time": datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat(),
    #         "terminal": 1,
    #         "call": { 
    #             "action": 2, 
    #             "destination": 2000,
    #             "allowed_lifts": [1001030]
    #             }
    #         }
    #     }

    #sandbox lift config
    # payload = {
    #     "type": "common-api",
    #     "buildingId": "building:HxKjGc3knnh",
    #     "requestId": "5e9e1d31-3d2c-42c7-907a-d8874117fb27",
    #     "callType": "config",
    #     "groupId": "1",
    #     }

    #sandbox lift monitoring
    # payload = {"type": "site-monitoring",
    #             "requestId": "1",
    #             "buildingId": buildingId,
    #             "callType": "monitor",
    #             "groupId": "1",
    #             "payload": {"sub": clientID, 
    #                         "duration": 300, 
    #                         "subtopics": ['lift_1/position','lift_2/position','lift_3/position','lift_1/doors','lift_2/doors','lift_3/doors']
    #                         },
    #             }
    
    # payload ={"type": "lift-call-api-v2",
    #             "buildingId": "building:PZFtaS27eW",
    #             "callType": "action",
    #             "groupId": "1",
    #             "payload": {"request_id": 252390420,
    #                         "area": 3000,
    #                         "time": "2022-03-10T07:17:33.298515Z",
    #                         "terminal": 1,
    #                         "call": { "action": 2, 
    #                                 }
    #                         }
    #         }

    
    # galenAdaptor.openLiftStateWS()
    # galenAdaptor.runSocketTilComplete_state()
    # galenAdaptor.sendLiftCommand(payload)
    # galenAdaptor.runSocketTilComplete()

    # galenAdaptor.liftDoorHoldingCall("L3", "C", 10)
    #galenAdaptor.liftLandingCall("L3","D")
    # galenAdaptor.openLiftConfigWS()
    # galenAdaptor.runSocketTilComplete_config()
    
if __name__ == '__main__':
    main()