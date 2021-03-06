#!/usr/bin/env bash
# Sets up a workspace in the current directory

set -e

# Get all the apt dependencies that are necessary
sudo apt-get update
sudo apt-get install -y \
        emacs25-nox \
        git \
        curl \
        libgsl-dev \
        libhdf5-dev \
        nano \
        portaudio19-dev \
        python-catkin-tools \
        python-rosinstall-generator \
        ros-melodic-amcl \
        ros-melodic-bfl \
        ros-melodic-easy-markers \
        ros-melodic-effort-controllers \
        ros-melodic-fetch-driver-msgs \
        ros-melodic-gazebo-ros \
        ros-melodic-joy \
        ros-melodic-kalman-filter \
        ros-melodic-map-server \
        ros-melodic-move-base \
        ros-melodic-moveit \
        ros-melodic-moveit-commander \
        ros-melodic-moveit-kinematics \
        ros-melodic-moveit-python \
        ros-melodic-moveit-ros \
        ros-melodic-moveit-ros-control-interface \
        ros-melodic-moveit-ros-visualization \
        ros-melodic-openni2-launch \
        ros-melodic-pcl-conversions \
        ros-melodic-pcl-ros \
        ros-melodic-robot-controllers \
        ros-melodic-slam-karto \
        ros-melodic-sound-play \
        ros-melodic-teleop-twist-keyboard \
        ros-melodic-trac-ik \
        sox \
        vim
which pip || (curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py && sudo python /tmp/get-pip.py)
sudo -H pip install -I -U -r requirements.txt

# Create the workspace directories
mkdir -p ./stable/src ./active/src

# If the repo has not been cloned, then clone it
if [ ! -d ./active/src/derail-fetchit ]; then
    git clone -b master git@github.com:GT-RAIL/assistance_arbitration.git ./active/src/assistance_arbitration
else
    echo "$(tput bold)Repository assistance_arbitration exists; Not cloning$(tput sgr0)"
fi

if [ ! -d ./active/src/derail-fetchit ]; then
    git clone -b banerjs/assistance-arbitration git@github.com:gt-rail-internal/derail-fetchit.git ./active/src/derail-fetchit
else
    echo "$(tput bold)Repository derail-fetchit exists; Not cloning$(tput sgr0)"
fi

# If the rosinstall files don't exist, setup the symlinks
if [ ! -f ./active/src/.rosinstall ]; then
    ln -s $(pwd)/active/src/assistance_arbitration/scripts/rosinstall/active.rosinstall ./active/src/.rosinstall
else
    echo "$(tput bold)Active workspace rosinstall file exists; Not linking$(tput sgr0)"
fi
if [ ! -f ./stable/src/.rosinstall ]; then
    ln -s $(pwd)/active/src/assistance_arbitration/scripts/rosinstall/stable.rosinstall ./stable/src/.rosinstall
else
    echo "$(tput bold)Stable workspace rosinstall file exists; Not linking$(tput sgr0)"
fi

# Initialize the workspaces
cd $(pwd)/stable/src/ && wstool up && cd $OLDPWD
cd $(pwd)/active/src/ && wstool up && cd $OLDPWD

# Then build the workspaces
source /opt/ros/melodic/setup.bash
cd $(pwd)/stable
catkin build
cd $OLDPWD

source ./stable/devel/setup.bash
cd $(pwd)/active
catkin build
catkin build --catkin-make-args run_tests && catkin_test_results
cd $OLDPWD
