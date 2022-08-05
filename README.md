# kone-ros-api
This is the KONE RMF Lift Adapter developed by Centre for Healthcare Assistive & Robotics Technology ([CHART]) based on KONE API v2.0. It acts as a translator in between RMF Lift messsages & KONE API v2.0.

This package has been tested with live KONE elevators at The Galen (Science Park 2, Singapore), with multiple elevators in a lift group (Lift A, Lift B, and Lift C).




Robotics Middleware Framework (RMF): https://github.com/open-rmf/rmf

KONE API v2.0: https://dev.kone.com/api-portal/dashboard/api-documentation/elevator-websocket-api-v2


## Pre-requisites
The following are packages/messages required by the adapter:
- [rmf_lift_msgs.msg]
- Python module: threading, requests, websocket, json

This packages has been tested to be working on:
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

## Fill up the config/env.yaml before you run this package
### 1. Get access_id and access_secret for KONE Elevator
To access KONE API, you will need access_id and access_secret. To control a live elevator, you will need to get these id  & secret from KONE Personnel. If you just want to test with their sandbox, then you can get id  & secret from [KONE API Portal].
### 2. Get building id
You can get the building id from KONE personnel, or getting the 'Resources Info' from KONE API (eg. GET /api/v2/application/self/resources.)

The format will be building:[BUILDING_ID] 

eg. building:HxKjGc3knnh

### 3. Fill up correct 'Scope'
At token_req_payload, the 'scope' will be in this format:

scope: robotcall/group:[BUILDING_ID]:1

eg. robotcall/group:[HxKjGc3knnh]:1

### 4. Put appropriate door holding time
There are 2 types of lift door holding time:
1. liftdoor_holding_duration_hard
2. liftdoor_holding_duration_soft


Hard means no matter what, it will open for full duration. (Max: 10 seconds)

Soft means door holding ends automatically by door sensor if anyone passes through. (Max: 30 seconds)

Note: Hard time will be counted first, then followed by soft time.


## Run the lift adapter
```
cd kone_ros_api_ws
source install/setup.bash
ros2 run kone_ros_api koneNode_v2 
```

## To test:
1. Echo RMF /lift_states to monitor the lift state
```
ros2 topic echo /lift_states
```
2. Send a RMF /lift_requests
```
ros2 topic pub /lift_requests rmf_lift_msgs/LiftRequest "{request_time: {sec: $EPOCHSECONDS}, lift_name: "C", session_id: "testing", request_type: 1, destination_floor: "L1", door_state: 2}" --once
```

## Notes:
Things to take note for KONE API v2.0:
1. For every authentication request posted, it will be expired in 3600s (1 hour). That's why koneNode getToken for every 3600s.
2. Liftstate websocket will be closed for every 300s (5 minutes). That's why koneNode reopen the liftstate websocket for every 300s.
3. Liftstate websocket will be closed if inactive for 60s (1 minute). That's why koneNode reopen the liftstate websocket if it is inactive for more thant 60s.

   [CHART]: <https://www.cgh.com.sg/Chart>
   [rmf_lift_msgs.msg]: <https://github.com/open-rmf/rmf_internal_msgs/tree/main/rmf_lift_msgs>
   [ROS2 Galactic]: <https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>
   [KONE API Portal]: <https://dev.kone.com/api-portal/dashboard>



