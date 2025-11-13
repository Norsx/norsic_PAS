mkdir ivan_pas

cd ivan_pas

git clone --branch zadatak1_01 --single-branch https://github.com/Norsx/norsic_PAS.git src

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash

ros2 launch robot_visualize display.launch.py
