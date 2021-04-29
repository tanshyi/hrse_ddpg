# hrse_ddpg


###################### Download and Install hrse_ddpg Package #######################

1. Change to the source space directory of the catkin workspace:

	$ cd ~/catkin_ws/src

2. Git Clone the relevant packages:

	$ git clone https://github.com/nicholashojunhui/hrse_ddpg.git

3. Build the packages in the catkin workspace:

	$ cd ~/catkin_ws && catkin_make

4. Go to catkin_ws/src/hrse_ddpg/turtlebot_ddpg/scripts/original_ddpg and catkin_ws/src/hrse_ddpg/turtlebot_ddpg/scripts/fd_replay/play_human_data, and make all python files executable



########### Installation Steps to install dependencies for hrse_ddpg ###############

1. Go to https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning and follow the steps to install Ananconda, ROS dependency packages, tensorflow and Keras

2. Follow the below instructions to uninstall and install numpy 

	$ pip uninstall numpy
	
	$ pip show numpy
	
	$ pip uninstall numpy
	
	$ pip show numpy
	
	$ pip install numpy pyqtgraph

3. install gym:

	$ pip install gym==0.10.5

4. Navigate to path: "~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf". Under "turtlebot3_waffle_pi.gazebo.xacro" file, right click and open with gedit.

	Set visual for laser = true:

		<xacro:arg name="laser_visual"  default="true"/>

	Set state -> LaserScan setting to 24 samples instead of 360
	
		<gazebo reference = "base_scan">
			<material>Gazebo/FlatBlack</material>
			<sensor type="ray" name="lds_lfcd_sensor">
			  <pose>0 0 0 0 0 0</pose>
			  <visualize>$(arg laser_visual)</visualize>
			  <update_rate>5</update_rate>
			  <ray>
			    <scan>
			      <horizontal>
			        <samples>24</samples>
	
