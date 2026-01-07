UPUTE
---------------------------------------------------
mkdir ivan_pas

cd ivan_pas

git clone --branch zadatak1_01 --single-branch https://github.com/Norsx/norsic_PAS.git src

source /opt/ros/humble/setup.bash

colcon build

# Z1---------------
source install/setup.bash
ros2 launch fanuc_m710ic_support view_robot.launch.py

# Z2---------------
Terminal1:
source install/setup.bash
ros2 launch fanuc_m710ic_support fanuc_controllers.launch.py

Terminal2:
source install/setup.bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]" --once

Promjena kontrolera:
Aktiviranje joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller

Postavljanje forward_position_controllera
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller

Joint trajectory:
source install/setup.bash
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
cd src/fanuc_m710ic_support/test
python3 test_trajectory.py

# Z3---------------
Terminal11:
source install/setup.bash
ros2 launch fanuc_m710ic_support fanuc_controllers.launch.py


Terminal2:
source install/setup.bash

Prebacivanje na forward_position_controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller

 	
Terminal3:
source install/setup.bash
ros2 launch fanuc_m710ic_support publish_forward_positions.launch.py

# Z4--------------
Terminal1:
source install/setup.bash
ros2 launch fanuc_m710ic_support fanuc_controllers.launch.py

Terminal2:
source install/setup.bash
Inicijalna provjera statusa kontrolera
ros2 control list_controllers
Aktivacija joint_trajectory_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
Konačna provjera da je promjena uspješna
ros2 control list_controllers

Terminal3:
source install/setup.bash
ros2 launch fanuc_m710ic_support publish_trajectory.launch.py
