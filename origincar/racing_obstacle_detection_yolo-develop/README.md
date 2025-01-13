English| [简体中文](./README_cn.md)

# Function Introduction

Identify obstacles in the track based on deep learning methods, using the YOLOv5s model.

# Instructions

## Preparation

Have a real robot or a robot simulation module with a motion base, camera, and RDK kit, and ensure they can operate normally.

## Install Package

**1. Install Package**

After starting the robot, connect to the robot via SSH or VNC through the terminal, click the "One-Click Deployment" button on this page's top right corner, copy and run the following commands on the RDK system to complete the installation of related nodes.

```bash
sudo apt update
sudo apt install -y tros-racing-obstacle-detection-yolo
```

**2. Run Obstacle Detection**

```shell
source /opt/tros/local_setup.bash
cp -r /opt/tros/lib/racing_obstacle_detection_yolo/config/ .

# Visualize obstacles on the web (open in the browser after starting the function ip:8000)
export WEB_SHOW=TRUE

# Simulation (use the simulation model)
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo_simulation.launch.py

# Actual Scenario (use the model in an actual scenario)
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo.launch.py

```


# Principle Overview

Horizon RDK acquires environmental data in front of the car through the camera. The image data is reasoned by the pre-trained YOLO model to obtain the image coordinates of obstacles and publish them.

# Interface Description

## Topics

### Published Topics

| Name                          | Message Type                                                 | Description                                             |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /racing_obstacle_detection    | ai_msgs/msg/PerceptionTargets             | Publishes the image coordinates of obstacles                 |

### Sub Topics
| Name                          | Message Type                                                     | Description                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hbmem_img or /image_raw       | hbm_img_msgs/msg/HbmMsg1080P or sensor_msgs/msg/Image        | Subscribes to the image messages published by the camera (640x480)                   |

## Parameters

| Parameter Name                | Type        | Description                                                                 |
| --------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| sub_img_topic       | string |     The topic name to subscribe to for images, configure according to the actual topic received, default value is /hbmem_img |
| is_shared_mem_sub   | bool | Whether to use shared memory, configure according to the actual situation, default value is True |
| config_file | string | Path for loading configuration file, configure according to the recognition situation, default value is config/yolov5sconfig_simulation.json |

# Note
This package provides models for obstacle detection in Gazebo simulation environment and specific real-world scenarios. If training with your own dataset, please remember to replace the models accordingly.
