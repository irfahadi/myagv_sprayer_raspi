# Source this on the Raspberry Pi (added to ~/.bashrc by install_raspi.sh)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/myagv_sprayer_raspi/src/myagv_sprayer_bringup/config/cyclonedds.xml"
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
[ -f "$HOME/myagv_sprayer_raspi/install/setup.bash" ] && source "$HOME/myagv_sprayer_raspi/install/setup.bash"
