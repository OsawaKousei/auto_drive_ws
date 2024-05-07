# update
sudo apt update
sudo apt upgrade -y

# environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/ && colcon build

# refresh bashrc
source ~/.bashrc
