#!/bin/bash

# Create log directory if it doesn't exist
mkdir -p logs

# Log file path
LOG_FILE="logs/installros2.log"

# Function to run a command and log its output
commands() {
    # Set locale
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    # Enable required repos
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    # install dev tools for ros2
    sudo apt update && sudo apt install ros-dev-tools -y
    # Install ROS 2
    sudo apt update
    sudo apt upgrade -y
    sudo apt install ros-jazzy-desktop -y
    # Install additional ROS 2 packages
    sudo apt-get install ros-jazzy-joint-state-publisher-gui -y
    sudo apt-get install ros-jazzy-xacro -y
    sudo apt-get install ros-jazzy-ros-gz* -y
    sudo apt-get install ros-jazzy-ros2-control -y
    sudo apt-get install ros-jazzy-ros2-controllers -y
    sudo apt-get install ros-jazzy-gz-ros2-control* -y
    sudo apt-get install ros-jazzy-moveit* -y
    # Install Aditional Linux Packages
    sudo apt-get install libserial-dev -y
    sudo apt-get install python3-pip -y
    sudo apt-get install python3-virtualenv -y
    # set a virtual environment for python
    # virtualenv -p python3 ./venv
    # touch ./venv/COLCON_IGNORE
    # source ./venv/bin/activate
    # Install Python Packages
    pip install -r requirements.txt
}

commands