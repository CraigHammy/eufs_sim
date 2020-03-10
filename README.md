# Heriot Watt Masters Group 1 Project

## 1. Contents of Repository
- [eufs_sim](https://gitlab.com/eufs/eufs_sim)
- [eufs_messages](https://gitlab.com/eufs/eufs_msgs)
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- fs_ai

## 2. Install Prerequisites 
- Install Ubuntu 16.04 LTS
- Install [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation)
- Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
- Download repo using
```bash 
git clone --recursive URL
```
- Install ROS dependencies by navigating to the catkin workspace and doing
```bash
rosdep install -i --from-path src/
```
- Install Python dependencies:
```bash
pip install -r eufs_gazebo/requirements.txt
```
- Install Masters Project Dependencies
```bash
sudo apt-get install ros-kinetic-perception-pcl
```

- (Optional but highly recommended) install [CUDA 9.0](https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=runfilelocal)

## 3. Compiling 

Navigate to your workspace and build the simulation:
```
cd [your-catkin-workspace]
catkin build
```
(add -DCMAKE_BUILD_TYPE=Release for packages such as darknet)

To enable ROS to find the packages you also need to run
```
source ./devel/setup.bash
```

_Note:_ source needs to be run on each new terminal you open. You can also include it in your `.bashrc` file.

## 4. Additional sensors
Additional sensors for testing are avilable via the `ros-kinetic-robotnik-sensor` package. Some of them are already defined in `eufs_description/robots/eufs.urdf.xarco`. You can simply commment them in and attach them appropriately to the car.


**Sensor suit of the car by default:**

- VLP16 lidar
- ZED Stereo camera
- IMU
- GPS
- odometry

An easy way to control the car is via
```
roslaunch ros_can_sim rqt_ros_can_sim.launch
```

(Make sure to modify the twist to ackerman package to publish to topic /cmd_vel_out)
