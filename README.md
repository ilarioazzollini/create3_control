# Create 3 Control

## How to Build

Clone the repository and start a Docker container

```
git clone https://github.com/ilarioazzollini/create3_control
cd create3_control
docker run -it --network=host --privileged -e DISPLAY=$DISPLAY -v $PWD:/root/create3_control irobotedu/create3-galactic bash
```

Build the repository

```
mkdir -p /root/control_ws/src
cd /root/control_ws
ln -s /root/create3_control src
colcon build
source install/setup.sh
```

Start the action server

```
python3 src/create3_control/create3_control/controller_server.py
```

Attach a new terminal to the Docker container and send a goal request to the server

```
ros2 action send_goal /drive_to_pose irobot_create_msgs/action/NavigateToPosition "{goal_pose:{pose:{position:{x: 1,y: 1.0,z: 0.0}, orientation:{x: 0.0,y: 0.0, z: 0.0, w: 1.0}}}}"
```
