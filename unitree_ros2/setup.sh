#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/foxy/setup.bash
source /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/unitree/unitree_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                            </Interfaces></General><Discovery>
                        </Discovery></Domain></CycloneDDS>'
# enable intel realsense permission
#sudo chmod 666 /dev/bus/usb/$(lsusb | grep -i intel | awk '{print $2"/"$4}' | sed 's/://')
