# uavusr_emulator
Small ROS publisher to output dummy data in the form of images, pose messages, and sensor readings

## Installing Dependencies
Before any steps can be made towards setting up the emulator, some packages must first be installed:

```sh
sudo apt install ros-kinetic-nav-msgs ros-kinetic-geometry-msgs ros-kinetic-image-transport ros-kinetic-std-msgs ros-kinetic-mavros-msgs ros-kinetic-sensor-msgs
```

## Compiling
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/uavusr_emulator
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Updating
```sh
roscd uavusr_emulator
git pull
cd ~/catkin_ws
catkin_make
```

## Running

#### Emulator

```sh
roslaunch uavusr_emulator emulator.launch
```

Additionally, a unique UAV name can be used when launching the emulator (replace `UAVNAME` with the desired name, defaults to "emulated_uav"):
```sh
roslaunch uavusr_emulator emulator.launch uav_name:=UAVNAME
```

Additional settings (such as position tracking speed and update rates) can be changed by editing the launch file itself (located in `uavusr_emulator/launch/emulator.launch`):

#### Waypoint Example
While the emulator itself does not come with an easy interface to control the current position with, the additional ROS package `contrail` can be used to perform all sorts of waypoint and tracking tasks. It is recommended that you refer to the `contrail` [documentation for further reference](https://github.com/qutas/contrail).

Some examples waypoints have been provided:
- `uavusr_emulator/movements/home.yaml`: a simple hover goal
- `uavusr_emulator/movements/land.yaml`: a simple landing goal
- `uavusr_emulator/movements/square.yaml`: a complex square path with changing heading

To run the waypoint tracking (replace `PATHNAME` with the desired waypoints, e.g. `home`):
```sh
roslaunch uavusr_emulator guidance.launch wp_name:=PATHNAME
```

Note, if you set a custom `UAVNAME`, you must also specify it with `uav_name:=UAVNAME` when running the waypoint guidance.

## Tuning the Simulation
Two files can be used to test the performance of tuning parameters:
- `uavusr_emulator/launch/emulator.launch`:
	- `model_mass`: Sets the weight of the model
	- `single_motor_thrust`: Sets the thrust (N) that a single motor can generate (quadrotor X4)
- `uavusr_emulator/config/mavel_params.yaml`: Refer to the [mavel documentation](https://github.com/qutas/mavel) for more details.

## Subscribed Topics
- Deploy Red Payload (std_msgs/Empty): `/UAVNAME/drop/red`
- Deploy Blue Payload (std_msgs/Empty): `/UAVNAME/drop/blue`

## Published Topics
#### UAV Information
- Autopilot State (mavros_msgs/State): `/UAVNAME/state`
- Battery Reading (sensor_msgs/BatteryState): `/UAVNAME/battery`
- Current Position (geometry_msgs/PoseStamped): `/UAVNAME/pose`
- Current Odometery (nav_msgs/Odometry): `/UAVNAME/odom`

#### Flight Area
- Realistic Map (nav_msgs/OccupancyGrid): `/grid/real`
- Test Map (nav_msgs/OccupancyGrid): `/grid/random`

#### Payload Feedback
- Red Deployment Position (geometry_msgs/PoseStamped): `/UAVNAME/drop/red/pose`
- Blue Deployment Position (geometry_msgs/PoseStamped): `/UAVNAME/drop/blue/pose`

#### Imagery
- Raw Image (sensor_msgs/Image): `/UAVNAME/image`
- Compressed Image (sensor_msgs/CompressedImage): `/UAVNAME/image/compressed`

