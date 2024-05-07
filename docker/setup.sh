# update
sudo apt update
sudo apt upgrade -y

# environment setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# create workspace
cd ~
git https://github.com/OsawaKousei/auto_drive_ws.git
cd auto_drive_ws/auto_drive
colcon build
echo "source ~/auto_drive/install/setup.bash" >> ~/.bashrc
cd ~

# refresh bashrc
source ~/.bashrc
