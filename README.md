# utilities_localisation

First Run Simulation:

DONT_RUN=1 make px4_sitl_default gazebo_uuv2

u und bestaetigen
dannach build ordner loeschen und folgende Befehle:

cd src/Firmware/Tools/sitl_gazebo/

git checkout hippocampus

git pull

cd ../..




DONT_RUN=1 make px4_sitl_default gazebo_uuv2

source ~/catkin_ws/devel/setup.bash    # (optional)

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 posix_sitl.launch




Copy file to pi


scp path_to_file pi@localization-pi: path to where to go
scp tags.yaml pi@localization-pi:~/catkin_ws/src/apriltag_ros/config/



