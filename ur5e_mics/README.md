# UR5e MICS 

Outline:

1. [Installation and dependencies](#installation)
2. [HowTo:](#howto)
	- [Use real robot with ROS](#real-robot)
	- [Use sim robot with ROS](#sim-robot)
3. [Run additional features](#additional-features)
4. [Contact](#contact)

This repo contains the packages to use the Merlin's Ur5e robot (simulation and real).

Summary of packages:
- _ur5e\_mics\_bringup_: launch real and sim robot
- _ur5e\_mics\_description_: URDFs of the cell
- _ur5e\_mics\_moveit\_config_: MoveIt configurations (motion planning and basic controllers)
- _ur5e\_mics\_configurations_: advanced controllers (use [cnr\_ros\_control](https://github.com/CNR-STIIMA-IRAS/cnr_ros_control/tree/master)). E.g., controllers for safety speed scaling
- _ur5e\_mics\_safety_: definition of HRC safety functions (e.g., safety areas) **(Wip)**
- _ur5e\_mics\_perception_: configurations of the sensors (e.g., cameras) **(TODO)**

## Installation and dependencies <a name="installation"></a>

**THIS SECTION IS WORK IN PROGRESS**

Before starting, make sure you:
- installed ros: follow the steps described in http://wiki.ros.org/ROS/Installation
- installed catkin_tools: follow the steps described in https://catkin-tools.readthedocs.io/en/latest/installing.html
- installed vcs: follow the steps described in http://wiki.ros.org/vcstool
- installed and configured rosdep: follow the steps described in http://wiki.ros.org/rosdep
- installed git and other depedencies:
  ```
  sudo apt install git  build-essential libqt5charts5-dev libqt5quickcontrols2-5 qtquickcontrols2-5-dev
  ```

Create a ROS workspace (if you already have one, you can skip this step):
```
mkdir -p ~/projects/merlin_ws/src
cd ~/projects/merlin_ws
catkin init
```
Then:
```
cd ~/projects/merlin_ws/src <--[use your workspace name] 
git clone https://github.com/MerlinLaboratory/ur5e_mics.git
mkdir rosinstall
cp ur5e_mics/ur5e_mics_base.repos rosinstall/ur5e_mics_base.repos 
vcs import < rosinstall/ur5e_mics_base.repos
cd ..
vcs pull src
sudo apt update && sudo apt upgrade -y
rosdep install --from-paths src --ignore-src -r -y
catkin config -j $(nproc --ignore=2)
catkin build -cs --mem-limit 50%
source devel/setup.bash
```

<!--
To use cnr\_ros\_controllers:

TODO

To use advanced motion planners:

TODO

-->

To use HRC safety, please clone:

- https://github.com/CNR-STIIMA-IRAS/ssm_safety
- https://github.com/CNR-STIIMA-IRAS/rosdyn


TODO: rosinstall

After installation, remember to

```
catkin build -cs && source {your_catkin_ws}/devel/setup.sh
```

## HowTo <a name="howto"></a>

### Use real robot with ROS <a name="real-robot"></a>

- Setup your PC:
	
	- set your PC's static IP to 192.168.150.10
	- connect the robot via Ethernet and ping the robot (192.168.150.1)

- Option1: Launch robot with basic ROS controllers:
	
	First, **switch robot to REMOTE CONTROL** on the teach pendant, then:

	```
	roslaunch ur5e_mics_bringup real_start.launch
	```

- Option2: Launch robot with _cnr\_ros\_controllers_:
	
	First, **switch robot to REMOTE CONTROL** on the teach pendant, then:

	```
	roslaunch ur5e_mics_configurations real_start.launch
	```

- Launch _ur\_scripts_ from ROS:
	
	It is possible to run robot scripts directly from ROS.
	You can define a robot program from the Teach Pendant and then call it with ROS services. This is useful for manipulation tasks in Cartesian space.

	**Services**:

	- Load program in the controller: 
	```/ur_hardware_interface/dashboard/load_program "filename: 'prova_moveL.urp'"```
	- Play loaded program: 
	```/ur_hardware_interface/dashboard/play```
	- Stop running program: ```/ur_hardware_interface/dashboard/stop```

	**IMPORTANT:** If you want to run a program and then go back to ROS control, remember to re-load 'ros1.urp' and play it.

#### Troubleshooting <a name="troubleshooting"></a>

If you have troubles moving the real robot with ROS please:
- try to ping the robot (192.168.150.1)
- if you are not able to ping the robot, check cable connectors and Ethernet cables
- make sure that **Ethernet/IP**  is **DISABLED** on the robot Installation panel
- make sure the robot is in **Remote Control** mode 
- ask Marco...

### Use sim robot with ROS <a name="sim-robot"></a>

- Launch simulated robot with basic ROS controllers:

	```
	roslaunch ur5e_mics_bringup fake_start.launch
	```

- Launch robot with _cnr\_ros\_controllers_:

	```
	roslaunch ur5e_mics_configurations fake_start.launch
	```

### Run additional features <a name="additional-features"></a>

- Launch HRC safety speed reduction: _see [here](./ur5e_mics_safety/)_

- Use Behavior Trees to launch high-level actions (e.g., pick, place, goto): _see [here]()_


### Contact <a name="contact"></a>

<img align="center" height="40" src="https://github.com/marco-faroni/marco-faroni.github.io/blob/master/images/marco_circle.png?raw=true"> Marco Faroni
