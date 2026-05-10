## Configuring RPI LAN connection
In order for the VMServer to be able to connect to the motors, the specified port needs to be open on the RPI LAN.
In Windows the simplest solution is to set the network to "private" as follows:
1. Open a powershell as Administrator
2. Find the name of the network connection win:
   > Get-NetConnectionProfile
3. Set network to private:
   > Set-NetConnectionProfile -Name "Unidentified network" -NetworkCategory Private
# UBT
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.0/24 dev eth0
sudo ip link set dev eth0 up
# RPI
sudo ip addr flush dev eth0
sudo ip addr add 192.168.137.2/24 dev eth0
sudo ip link set dev eth0 up
sudo ip route add default via 192.168.137.1 dev eth0
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null
cat << 'EOF' > /home/foo/fastdds_rpi.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_eth0</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>192.168.137.2</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>udp_eth0</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
export FASTDDS_DEFAULT_PROFILES_FILE=/home/foo/fastdds_rpi.xml
unset FASTRTPS_DEFAULT_PROFILES_FILE
export FASTDDS_DEFAULT_PROFILES_FILE=/home/foo/fastdds_rpi.xml

## Setup SLAM system
An Ubuntu-24.04 distro with the relevant code needs to be added and the windows system needs to be configured:
1. Install the dist: (Windows CMD)
   > wsl --import Ubuntu-24.04 [path to Ubuntu-24.04.tar]
2. Configure WSL: (Windows powershell as administrator)
   > "@
     autoProxy=true
     firewall=true
     dnsTunneling=true
     networkingMode=mirrored
     [wsl2]
     $wslConfig = @"
   > $wslConfig | Set-Content -Path "$HOME\.wslconfig"

## Updating RPI code
> scp .\PRODUCT\RPI\* foo@ood:~/Desktop/PRESENT/

## Testing ROS2 mapping
# RPI
cd ~/ros2_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash
sudo chmod 666 /dev/ttyUSB0
1. ros2 launch robot_bringup record_c1.launch.py
2. ros2 node list && ros2 topic list && ros2 topic echo /scan
# PC
wsl -d Ubuntu-24.04
cd ~/cartographer_ws
1. > ros2 launch my_robot_slam online_slam_v2.launch.py
2. > ros2 bag record -s mcap -a -o test2
3. > ros2 run rviz2 rviz2 -d ~/cartographer_ws/src/my_robot_slam/rviz/mapper.rviz

