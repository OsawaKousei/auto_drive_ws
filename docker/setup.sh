# update
sudo apt update
sudo apt upgrade -y

# environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# create workspace
cd ~
git clone https://gitlab.com/kikaiken/auto_drive_ws.git
cd auto_drive_ws/auto_drive
colcon build
echo "source ~/auto_drive_ws/auto_drive/install/setup.bash" >> ~/.bashrc
cd ~

# refresh bashrc
source ~/.bashrc
