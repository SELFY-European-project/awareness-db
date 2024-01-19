# SELFY Project - Multi-Agent Situational Awareness Database

This repository contains the data of the *Multi-Agent Situational
Awareness Database* produced in the scope of the [SELFY
project](https://selfy-project.eu/) and corresponding to the deliverable
D3.2 of this project.

The SELFY project has received funding from the Horizon Europe programme
under grant agreement No. 101069748. This work reflects only the
author's view. Neither the European Commission nor the CINEA is
responsible for any use that may be made of the information it contains.

This dataset contains awareness data from different sensors (hence
"multi-sensor" i.e., an agent can employ several sensors: camera, LiDAR,
GNSS, etc) and recorded on various C-ITS agents (hence "multi-agent":
vehicles, RSUs, other road users and C-ITS infrastructure equipment). It
contains sensor measurements with associated calibration parameters and
the associated ground truth when possible. 



## Database content

This section gives high level information about the content of the
dataset. For more detailed information, please refer to the deliverable
D3.2 of SELFY.

The dataset includes both real measurements and synthetic data generated
with the [CARLA simulator](https://carla.org/). It contains the
following type of data: 

* Images from video cameras looking at the scene,
* Cloud points from a LiDAR,
* The record of an RTK-GNSS receiver installed on the vehicle and
  ground truth obtained from 2D/3D annotations as bounding boxes edited
  by a human,
* The record of an RTK-GNSS receiver installed on the head of the
  pedestrian(s) and ground truth obtained from 2D/3D annotations as
  bounding boxes edited by a human.


## Database format

For the moment, the data is provided under the form of [ROS1
Rosbags](http://wiki.ros.org/Bags/Format/1.2). We aim to move to [ROS2](http://wiki.ros.org/Bags/Format/2.0) in a short future but it is not yet possible. 

The synthetic data and real-measurement follow as much as possible the
same message specification even if some specificities remains for each
kind of data.

The synthetic data lies in the `simulation/` directory while the
`real-world/` directory contains the real measurements.