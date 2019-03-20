# hercules robot 

Skid-steer drive robot using [hercules chassis](https://www.seeedstudio.com/Skeleton-Bot-4WD-Hercules-Mobile-Robotic-Platform-p-1504.html). 

# Requirements

* ROS 

# Startup

1. Launch skid steer drive interface with motors
```bash
rosrun hercules_base skid_steer_drive.py
```

2. Launch ROS closed loop control
```bash
roslaunch hercules_control control.launch
```

3. Launch teleop
```bash
roslaunch hercules_control teleop.launch
```
