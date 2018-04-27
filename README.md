# uavusr_emulator
Small ROS publisher to output dummy data in the form of images, pose messages, and sensor readings

## Compiling
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/uavusr_emulator
cd ../
catkin_make
```

## Running
```
roslaunch uavusr_emulator emulator.launch
```

## Subscribed Topics
- **Position Setpoint** (geometry_msgs/PoseStamped): ~/uavusr/goal
- **Deploy Red Payload** (std_msgs/Empty): ~/drop/red
- **Deploy Blue Payload** (std_msgs/Empty): ~/drop/blue

## Published Topics
#### UAV Information
- **Current Position** (geometry_msgs/PoseStamped): ~/uavusr/pose
- **Current Velocity** (geometry_msgs/TwistStamped): ~/uavusr/twist

#### Flight Area
- **Realistic Map** (nav_msgs/OccupancyGrid): ~/grid/real
- **Test Map** (nav_msgs/OccupancyGrid): ~/grid/random

#### Payload Feedback
- **Red Deployment Position** (geometry_msgs/PoseStamped): ~/drop/red/pose
- **Blue Deployment Position** (geometry_msgs/PoseStamped): ~/drop/blue/pose

#### Imagery
- **Raw Image** (sensor_msgs/Image): ~/image
- **Compressed Image** (sensor_msgs/CompressedImage): ~/image/compressed

