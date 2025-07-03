# digital_twin

contains urdf file of the batmobile robot

lauch model_motor.launch.py to start rviz2 and gazebo simulation
terminal command:

- ros2 lauch model model_motor.launch.py

Test send command to model and robot

- ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 250.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once



-------------------

Run on robot

See if messages arrive
- ros2 topic echo /motor_command

Run driver node: 

- ros2 run serial_motor_demo driver --ros-args -p serial_port:=/dev/ttyACM0 -p baud_rate:=57600 -p loop_rate:=30