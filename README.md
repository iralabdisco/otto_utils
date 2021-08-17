# Otto Utils

Otto's nodes and launch files that do not belong elsewhere and don't deserve their own package.

## Description

### Otto PID tuning
The node ``` otto_pid_tuning.py ``` is used to tune PID parametes.
It subscribes to ```cmd_vel``` and ```otto_ticks``` and publishes a ```otto_pid_tuning_val``` message.
You can use ```rqt_plot``` to plot the values and tune your PID.

### Odom Laser Calibration
The node ```otto_odom_laser_calib.py``` is used to calibrate the odometry parameters and the laser pose.
It subscribes to ```otto_ticks``` and publishes the left and right wheel velocities (in radians/s) on ```/otto_odom_calib``` topic.
The data published on ```/otto_odom_calib```, together with the data on ```/scan``` topic can be used with the package  [OdomLaserCalibraTool](https://github.com/MegviiRobot/OdomLaserCalibraTool) to automagically get the wheels radius, the distance between the wheels and the laser pose.

### Joypad
The node ```joy_to_cmd_vel.py``` subscribes to ```joy``` and publishes a ```cmd_vel``` msg.
