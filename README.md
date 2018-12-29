This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


![Screenshot](https://github.com/cazacov/CarND-Capstone/blob/master/imgs/screenshot.jpg?raw=true)

## Smooth Acceleration Profile

In reallity the car will decelerate in order to stop, starting with initial velocity v0. To make calculations simplier, it's convenient to reverse the time and think car is accelerating from initial v = 0 to final v = v0 in T seconds.

Simple waypoint velocity calculation algorithm proposed at Udacity project walkthrough is easy to implement, but it has one drawback: the car starts to brake abruptly causing high jerk that is not comfortable for passengers:

![Constant acceleration](https://github.com/cazacov/CarND-Capstone/blob/master/imgs/constant_acceleration.png?raw=true)

To reduce maximum jerk I decided to take as acceleration function a part of sine wave that has max slope of 1.

![Sine wave](https://github.com/cazacov/CarND-Capstone/blob/master/imgs/sine_profile.png?raw=true)

In case of smooth deceleration profile the velocity at the end of braking distance is close to 0 with slope also close to 0 and that makes the PID controller unstable. The calculated velocity tends to oscilate around 0 sometimes getting small negative values. In simulator that means the car stops before the stopline, waits a little and then again moves couple of centimeters forward before stopping completely. Well, when I just got my driving license it was probably the way I was driving, but we want the smart car be better than a newbie human.

To make velocity curve more PID-friendly I decided to take only the middle 80% of it, cutting 10% on both sides. After calculating derivatives and antiderivatives I got the following formulae:

![Math](https://github.com/cazacov/CarND-Capstone/blob/master/imgs/acceleration_profile_math.png?raw=true)

![Math](https://github.com/cazacov/CarND-Capstone/blob/master/imgs/smooth_acceleration.png?raw=true)

It can be easily proved that s(t) is bijective and must have inverse function in the range [0..T], but unfortunately it cannot be expressed in terms of standard matematical functions. In code I solve this problem numerically just incrementing time in samll steps till I get the desired distance. Then, knowing the time, I can easily calculate velocity at that waypoint (waypoint_updater.py lines 155-158).

## Traffic Light Detector

## ROS Topic with Detector Output

The Traffic Light Detection node was extended with a new topic "/traffic_light_detected", message type: sensor_msgs/Image. After processing camera image the traffic light classifier creates a new image with marked traffic lights and their color. That is useful for debugging puposes. To view topic message stream in real type run
```bash
rqt_image_view /traffic_light_detected
```
Here on the screen cas you can see two image streams on the right hand side. The top one is raw camera output as provided by simulator and at the bottom there is the processed version:

![Screencast](https://github.com/cazacov/CarND-Capstone/blob/master/imgs/anim.gif?raw=true)


## Setup
Please use **one** of the two installation options, either native **or** docker installation.

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
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

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

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

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

3. Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) and dbw-mkz package
```bash
sudo apt-get install ros-melodic-dbw-mkz
```

4. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
5. Run the simulator

6. (Optional) View camera video stream
```bash
rqt_image_view /image_color
```



### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images



### Usefull ROS commands
List topics
```bash
rostopic list
```

Get topic info
```bash
rostopic info /image_color
```

Get message info
```bash
rosmsg info sensor_msgs/Image
```

Echo topic
```bash
rostopic echo /traffic_waypoint
```


