# Repository Contents
This is part of Team 1's final project for MAE/ECE 148 WI26, and is based off of Team 7's project. The repository is already set up in the public docker container.

This repository contains a ROS2 package used for combining GPS and OAK-D object tracking outputs to follow a GPS path using VESC controls while overtaking opponent robocars.

Inside the tracker_v2 folder:
- gps_publisher, yolo_oakd_tracking, and gps_path_follow_node are created by our team
- vesc_twist_node and the accompanying vesc submodule are created by Team 7
- only these 4 nodes are used, the rest are created by Team 7 and are not used for our project.

To run, simply run the launch_ros2 bash command (which you can inspect in ~/.bashrc), which builds and sources everything necessary. 

# Docker Setup

(originally written by Team 7)

Assuming x11 installed and working on RPI and Laptop


```bash
ssh -X -Y user@rpihostname
```

Fix X11 permissions
```bash
xhost +local:root
```

set udev rules:
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

pull docker container:
```bash
docker pull perilune/ros2_robocar_tracking:final_proj
```

proper docker command:
```bash
docker run \
    --name ros2_doc \
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    --volume /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --device /dev/video0 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --volume /etc/passwd:/etc/passwd:ro \
    --volume /etc/group:/etc/group:ro \
    perilune/ros2_robocar_tracking:final_proj
```

