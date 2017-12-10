[![Waffle.io - Columns and their card count](https://badge.waffle.io/SDC-Team-LastMinute/CarND-Capstone.png?columns=all)](https://waffle.io/SDC-Team-LastMinute/CarND-Capstone?utm_source=badge)
# Team LastMinute

### Team members:

- Alistair Kirk	- kirkas@shaw.ca
- Ashish Tyagi	- ashishtyagi.mi@gmail.com
- Joe Orvidas	- jorvidas@gmail.com
- Varun Pandey	- varunpandey0502@gmail.com
- Marko Kurtz (Team Lead) - marko.kurtz@gmail.com

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### TODO: project description ...
### Nodes:
#### Waypoint Updater

#### DBW

#### Traffic Light Detection

When approaching a traffic light, the vehicle needs to recognize the color of the light and act accordingly.  The traffic light detection node uses a neural network classifier to determine if there is a light, what state it is in, and then chooses the correct stopping position for the light from a predetermined list of positions.

In order to do this, it takes in continuously updated feeds on the vehicles pose and images from the car's cameras. This is combined with known information of the waypoints along the road and positions to stop for each stop light. With this information, the traffic light detection node tries to find the closest traffic light stop point in view, if it finds one, it classifies the lights state and returns both the nearest stop point and the traffic light state.

**Additional Notes on Traffic Light Detection**
* Idea to use tensorflow object api came from - https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58
* Network model choice based in inference speed and initial pretraining accuracy of detecting traffic light on udacity sim and real course images.
	* Idea was to try models from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md and pick one that has good inference time with good accuracy - detecting traffic lights out of the box. After some experimentation, the faster_rcnn_inception_v2_coco model was chosen.
	* Sample traffic light data was collected from both the simulator and from udacity provided bags for the real course - capturing was done using image_exporter from ros while running the simulator as well as playing before mentioned udacity rosbags. Dataset preparation was done according to https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md. Dataset specs (add table with number of samples)
	* NN for real and sim were trained separately, due to inferencing issues when trained together. Training was done with transfer learning using already pretrained faster_rcnn_inception_v2_coco model (started from models fine_tune_checkpoint - https://github.com/tensorflow/models/blob/f87a58cd96d45de73c9a8330a06b2ab56749a7fa/research/object_detection/g3doc/configuring_jobs.md)

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
