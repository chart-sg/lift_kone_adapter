# kone-ros-api
This KONE RMF Lift Adapter is developed by Centre for Healthcare Assistive & Robotics Technology ([CHART]) based on KONE API v2.0. It acts as a translator in between [RMF] [Lift Messages] & [KONE API v2.0]. 

There are 3 main nodes/files:
1. [koneNode_v2.py]
   - subscribe ros2 RMF [/lift_requests] topic from RMF core
   - publish ros2 RMF [/lift_states] topic to RMF core

2. [koneAdapter_v2.py]
   - send various lift commands to KONE API v2.0 via websocket
   - get lift state from KONE API v2.0 via websocket

3. [env.yaml] in 'config' folder 
   - config file for KONE lift parameter: 
      - access_id
      - access_secret
      - building_id
      - connectionURL
      - baseURL
      - auth_server_url
      - requestHeaders
      - token_req_payload
      - lift_group_to_control
      - ws_url
      - liftdoor_holding_duration_hard
      - liftdoor_holding_duration_soft


*This package has been tested with live KONE elevators at [The Galen] (Science Park 2, Singapore), with multiple elevators in same lift group (with Lift A, Lift B, Lift C in group 0).

## Pre-requisites
The following are packages/messages required by the adapter:
- [rmf_lift_msgs.msg]
- Python module: threading, requests, websocket, json

This package has been tested to be working on:
- Ubuntu 20.04
- [ROS2 Galactic]

## Installation
```
mkdir -p kone_ros_api_ws/src
git clone https://github.com/sharp-rmf/kone-ros-api.git
cd ..
source YOUR_RMF_Lift_MSG_WORKSPACE (eg. source ~/rmf_ws/install/setup.bash)
colcon build
```
## Dockerfile
To quickly start running your own Kone RMF Lift Adapter with the Kone v2 API. Simply copy the docker file out, build it, and run it with the correct env.yaml file (which contains the secrets).
> If you get a <code>KeyError: 'access_token'</code> error, please check that your env.yaml's <code>access_id/access_secret</code> is correct.
```
docker build -f src/kone-ros-api/Dockerfile -t kone --build-arg GIT_VER=6e75d1c18d1132bf8bacd51b1fee29ee9b2dab42 . # To checkout git commit 6e75d1
docker run -it --network host --mount type=bind,source=$PWD/src/kone-ros-api/config/env.yaml,destination=/opt/rmf/install/kone_ros_api/share/kone_ros_api/config/env.yaml kone:latest /bin/bash
```

## Fill up the config/env.yaml before you run this package
### 1. Get access_id and access_secret for KONE Elevator
To access KONE API, you will need access_id and access_secret. 

To control a live elevator, you will need to get these id  & secret from KONE Personnel. 

If you just want to test with their sandbox, then you can get id  & secret from [KONE API Portal] --> 'Create Application'--> 'Sandbox'.

### 2. Get building id
You can get the building id from KONE personnel, or getting the '[resources]' via KONE API call (eg. GET /api/v2/application/self/resources.)

The format will be building:[BUILDING_ID] 

eg. building:HxKjGc3knnh

### 3. Fill up correct 'Scope'
At token_req_payload, the '[scope]' will be in this format:

scope: robotcall/group:[BUILDING_ID]:1

eg. robotcall/group:[HxKjGc3knnh]:1

### 4. Put appropriate 'Door Holding Time'
There are 2 types of lift [door holding time]:
1. liftdoor_holding_duration_hard
2. liftdoor_holding_duration_soft


Hard means no matter what, it will open for full duration. (Max: 10 seconds)

Soft means door holding ends automatically by door sensor if anyone passes through. (Max: 30 seconds)

Notes: 
1. Hard time will be counted first, then followed by soft time.
2. If you want to set longer than 40 seconds, then you will need to send multiple door holding command (each with a max of 10s + 30s = 40s).


## Run the lift adapter
```
cd kone_ros_api_ws
source install/setup.bash
ros2 run kone_ros_api koneNode_v2 
```

## To test:
1. Echo ros2 RMF /lift_states topic to monitor the lift state
```
ros2 topic echo /lift_states
```
2. Send a ros2 RMF /lift_requests topic
```
ros2 topic pub /lift_requests rmf_lift_msgs/LiftRequest "{request_time: {sec: $EPOCHSECONDS}, lift_name: "C", session_id: "testing", request_type: 1, destination_floor: "L1", door_state: 2}" --once
```

## Notes:
Things to take note for KONE API v2.0:
1. For every [authentication request] posted, it will be expired in 3600s (1 hour). That's why koneNode getToken for every 3600s.
2. Liftstate websocket will be closed by KONE Server if it has been opened for more than 300s (5 minutes). That's why koneNode reopens the liftstate websocket for every 300s.
3. Liftstate websocket will be closed by KONE Server if inactive for 60s (1 minute). That's why koneNode reopens the liftstate websocket if it is inactive for more than 60s.

## Useful links:
1. Robotics Middleware Framework (RMF): https://github.com/open-rmf/rmf
2. KONE API v2.0: https://dev.kone.com/api-portal/dashboard/api-documentation/elevator-websocket-api-v2


   [RMF]: <https://github.com/open-rmf/rmf>
   [Lift Messages]: <https://github.com/open-rmf/rmf_internal_msgs/tree/main/rmf_lift_msgs>
   [KONE API v2.0]: <https://dev.kone.com/api-portal/dashboard/api-documentation/elevator-websocket-api-v2>
   [CHART]: <https://www.cgh.com.sg/Chart>
   [rmf_lift_msgs.msg]: <https://github.com/open-rmf/rmf_internal_msgs/tree/main/rmf_lift_msgs>
   [ROS2 Galactic]: <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>
   [KONE API Portal]: <https://dev.kone.com/api-portal/dashboard>
   [resources]: <https://dev.kone.com/api-portal/dashboard/api-documentation/authentication-api-v2#listResources>
   [scope]: <https://dev.kone.com/api-portal/dashboard/developer-guide/overview-api#scopes>
   [door holding time]: <https://dev.kone.com/api-portal/dashboard/api-documentation/elevator-websocket-api-v2/robots#hold-car-door-open>
   [authentication request]: <https://dev.kone.com/api-portal/dashboard/api-documentation/elevator-websocket-api-v2/robots#authentication>
   [koneAdapter_v2.py]: <https://github.com/sharp-rmf/kone-ros-api/blob/main/kone_ros_api/koneAdaptor_v2.py>
   [koneNode_v2.py]: <https://github.com/sharp-rmf/kone-ros-api/blob/main/kone_ros_api/koneNode_v2.py>
   [env.yaml]: <https://github.com/sharp-rmf/kone-ros-api/blob/main/config/env.yaml>
   [The Galen]: <https://www.capitaland.com/en/find-a-property/global-property-listing/businesspark-industrial-logistics/the-galen.html>
   [/lift_requests]: <https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg>
   [/lift_states]: <https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftState.msg>
