#Install ros-noetic on ubuntu 20.04
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-roslaunch python3-pip -y
sudo rosdep init
rosdep update
#Install dependence package
sudo apt install ros-noetic-rplidar-ros ros-noetic-realsense2-camera ros-noetic-rtabmap-ros ros-noetic-rtabmap ros-noetic-octomap-rviz-plugins ros-noetic-navigation ros-noetic-velodyne-pointcloud -y
pip install pyads
#Install VSCode
sudo snap install --classic code
#Install filezilla
sudo apt install filezilla -y
#Install teamviewer
wget https://download.teamviewer.com/download/linux/teamviewer_amd64.deb
sudo apt install ./teamviewer_amd64.deb
