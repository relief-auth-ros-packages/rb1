#!/bin/bash
# The BSD License


#**********************************************USAGE *************************************************
function usage {
    # Print out usage of this script.
    echo >&2 "usage: $0 [catkin workspace name (default:catkin_ws)] [ROS distro (default: indigo)"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}
#*****************************************************************************************************

# Parse command line. If the number of argument differs from what is expected, call `usage` function.
OPT=`getopt -o h -l help -- $*`
if [ $# != 2 ]; then
    usage
fi
eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

name_catkinws=$1
name_catkinws=${name_catkinws:="catkin_ws"}
name_ros_distro=$2
name_ros_distro=${name_ros_distro:="indigo"}
#arduimu = $3
#gyro = $4

version=`lsb_release -sc`

echo "[Checking the ubuntu version]"
case $version in
  "saucy" | "trusty")
  ;;
  *)
    echo "[This script will only work on ubuntu saucy(13.10) or trusty(14.04)]"
    exit 0
esac

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing Standard tools]"
sudo apt-get install openssh-server vim subversion git apache2 mingetty terminator -y

echo "[Making the sources dir]"
mkdir -p ~/sources

echo "[Copying git packages and Linking]"
cd ~/sources
git clone https://github.com/RobotnikAutomation/rb1_base_robot.git
git clone https://github.com/RobotnikAutomation/robotnik_msgs.git #CAN BE INSTALLED FROM SOURCES sudo apt-get install ros-indigo-robotnik-msgs
git clone https://github.com/RobotnikAutomation/robotnik_sensors.git
git clone https://github.com/RobotnikAutomation/rb1_base_common.git

echo "[Check the 14.04.2 TLS issue]"
relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
if [ "$relesenum" = "14.04.2" ]
then
  echo "Your ubuntu version is $relesenum"
  echo "Intstall the libgl1-mesa-dev-lts-utopic package to solve the dependency issues during the ROS installation"
  sudo apt-get install -y libgl1-mesa-dev-lts-utopic
else
  echo "Your ubuntu version is $relesenum"
fi

echo "[Installing chrony and setting the ntpdate]"
sudo apt-get install -y chrony
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing ROS]"
sudo apt-get install -y ros-$name_ros_distro-desktop-full ros-$name_ros_distro-rqt-*

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/$name_ros_distro/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/$name_catkinws/src
cd ~/$name_catkinws/src
catkin_init_workspace
cd ~/$name_catkinws/
catkin_make

echo "[Installing Standard tools]"
sudo apt-get install sixad -y

echo "[Setting the ROS evironment]"
sh -c "echo \"source /opt/ros/$name_ros_distro/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkinws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc" #FOR RB1-BASE ROS_MASTER_URI=http:// q:11311
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc" #FOR SUMMIT ROS_HOSTNAME=192.168.0.200

echo "[Configure .bashrc]"
cd
printf '\n#RB1-BASE \n#AUTOBOOT \necho "ROBOTNIK RB1-BASE" \n\tTerminal=`tty` \n\tcase $Terminal in \n\t\t"/dev/tty2") roscore;; \n\t\t"/dev/tty3") sleep 20; \n\t\troslaunch rb1_base_bringup rb1_base_complete.launch;; \n\tesac\n' >> .bashrc
sudo printf '\n# ROOT \n# AUTOBOOT \necho "ROBOTNIK RB1-BASE" \n\tTerminal=`tty` \n\tcase $Terminal in \n\t\t"/dev/tty1") sleep 5; \n\t\tsixpair; \n\t\tsixad start;; \n\tesac\n' >> /root/.bashrc

echo "[Configuring tty]"
sudo sed -i.bak 's/exec \/sbin\/getty -8 38400 tty1/exec \/sbin\/mingetty --autologin root tty1/g' /etc/init/tty1.conf
sudo sed -i.bak 's/exec \/sbin\/getty -8 38400 tty2/exec \/sbin\/mingetty --autologin rb1 tty2/g' /etc/init/tty2.conf
sudo sed -i.bak 's/exec \/sbin\/getty -8 38400 tty3/exec \/sbin\/mingetty --autologin rb1 tty3/g' /etc/init/tty3.conf

echo "[Configuring ACPI options]"
sudo sed -i.bak 's/action=\/etc\/acpi\/powerbtn.sh/#action=\/etc\/acpi\/powerbtn.sh\naction=\/sbin\/poweroff/g' /etc/acpi/events/powerbtn

echo "[Configuring GRUB options]"
sudo sed -i.bak 's/GRUB_CMDLINE_LINUX=""/GRUB_CMDLINE_LINUX=""\nGRUB_RECORDFAIL_TIMEOUT=0/g' /etc/default/grub

echo "[Adding user rb1 to root and dialout]"
sudo usermod -a -G dialout rb1 && sudo usermod -a -G root rb1

echo "[ROS Extra packages]"
sudo apt-get install ros-indigo-transmission-interface ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-navigation ros-indigo-ros-control ros-indigo-ros-controllers ros-indigo-velocity-controllers ros-indigo-control-toolbox ros-indigo-cmake-modules ros-indigo-serial ros-indigo-joystick-drivers ros-indigo-rosbridge-server ros-indigo-robot-localization ros-indigo-twist-mux ros-indigo-rosbridge-suite ros-indigo-imu-tools ros-indigo-mavros-* ros-indigo-moveit-* -y

echo "[Installing PCAN Driver]"
cd ~/Downloads && wget --quiet http://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-7.15.2.tar.gz
sudo apt-get install libpopt-dev -y
tar -xzf peak-linux-driver-7.15.2.tar.gz && cd peak-linux-driver-7.15.2
make clean
make NET=NO_NETDEV_SUPPORT
sudo make install

echo "[Linking & Recompiling after install PEAK-CAN]"
cd ~/$name_catkinws/src
ln -sf ~/sources/robotnik_sensors && ln -sf ~/sources/robotnik_msgs && ln -sf ~/sources/rb1_base_common && ln -sf ~/sources/rb1_base_robot
cd ~/$name_catkinws/
catkin_make

echo "[Pairing the PAD]"
sudo sixpair
sudo sixad --boot-yes

echo "[Complete!!!]"

exec bash

exit 0

