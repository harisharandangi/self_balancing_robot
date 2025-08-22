This Project includes,

  Real-time balance control using IMU feedback
	
  PID-based stabilization
	
  Forward motion bias (`forward_speed` parameter)
	
  ROS integration with `/cmd_vel` for robot control
	

	
Prerequirements

  Ubuntu 20.04 OS with 
	
    Ros installation (ROS 1 Noetic)
		
    Gazebo Simulation Environment
		

		
Running Selfbalancing Robot Simulation

  cd ~/catkin_ws/src
	
  git clone https://github.com/<harisharandangi>/self_balancing_robot.git
	
  cd ~/catkin_ws

  catkin_make
	
  source devel/setup.bash

  roslaunch self_balancing_robot robot_gazebo.launch

	
(Optional) Set forward speed:

  rosrun self_balancing_robot script.py _forward_speed:=0.3

	
Topics

  /imu/data → IMU sensor readings
	
  /cmd_vel → velocity commands to wheels

	
The PID controller is defined in balance_controller.py:

  Kp = 15.0
	
  Ki = 6.0
	
  Kd = 0.001
