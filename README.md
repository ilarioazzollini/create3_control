docker run -it --network=host --privileged -e DISPLAY=$DISPLAY -v $PWD:/root/create3_control irobotedu/create3-galactic bash

root@ilo-pc:~# mkdir -p control_ws/src
root@ilo-pc:~# cd control_ws/
root@ilo-pc:~/control_ws# ln -s /root/create3_control src

ros2 action send_goal /drive_to_pose irobot_create_msgs/action/NavigateToPosition "{goal_pose:{pose:{position:{x: 1,y: 0.0,z: 0.0}, orientation:{x: 0.0,y: 0.0, z: 0.0, w: 1.0}}}}"
