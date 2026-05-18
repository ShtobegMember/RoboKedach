# Setup UBT on WLS for SLAM system
An Ubuntu-24.04 distro with the relevant code needs to be added and the windows system needs to be configured:
1. Install the dist: (Windows CMD)
```
wsl --import Ubuntu-24.04 [path to project root (not git root!)] [path to Ubuntu-24.04.tar]
```

# Configuring RPI-UBT-WDS LAN connection
## WDS ip configuration
1. Open a powershell as Administrator
2. Find the name of the network connection with:
```
netsh interface ip show config
```
3. Update the ip:
```
netsh interface ip set address "Ethernet 2" static 192.168.1.1 255.255.255.0
```
## WDS LAN set to private
In order for the VMServer to be able to connect to the motors, the specified port needs to be open on the RPI LAN.
In Windows the simplest solution is to set the network to "private" as follows:
1. Open a powershell as Administrator
2. Find the name of the network connection with:
```
Get-NetConnectionProfile
```
3. Set network to private:
```
Set-NetConnectionProfile -Name "Unidentified network" -NetworkCategory Private
```
## WDS firewall
Set inbound rull open to any protocal on any ports, for "private" networks, and for ip's:
198.168.1.0, 198.168.1.1, 198.168.1.2
## WDS virtual bridge
In search bar: "Turn windows features on or off" > check "Hyper-V" > press "ok"  
After restart, in search bar "Hyper-V Manager" > Virtual Switch Manager > "External" >  
As external network choose the Ethernet connection connecting to the RPI, as switch name enter "WSL_Bridge"
## UBT
1. Configure WSL: (Windows powershell as administrator)
```
$wslConfig = @"
[wsl2]
networkingMode=bridged
vmSwitch=WSL_Bridge
"@
$wslConfig | Set-Content -Path "$HOME\.wslconfig"
```
2. Configure UBT ip
```
wsl -d Ubuntu-24.04
sudo ip addr flush dev eth0 (sudo password: " ")
sudo ip addr add 192.168.1.0/24 dev eth0
sudo ip link set dev eth0 up
```
## RPI
```
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.2/24 dev eth0
sudo ip link set dev eth0 up
```
# Debugging Mapping
## RPI-UBT-WDS LAN connection
Can all three machines ping eachother? Check:
1. .wslconfig correctly configured
2. Hyper-V switch correctly configured
3. UBT eth0 iterface has correct ip and mask
4. Is WDS firewall inbound rule configured correctly
## Testing ROS2 talker-listener connection
### RPI
```
ssh foo@ood (password: " ")
cd ~/ros2_ws
source /opt/ros/kilted/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
sudo chmod 666 /dev/ttyUSB0
ros2 run demo_nodes_py talker
```
### UBT
```
wsl -d Ubuntu-24.04
cd ~/cartographer_ws
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_py listener
```
## Testing ROS2 sensor topics connection
### RPI
```
*setup commands as above*
ros2 launch robot_bringup record_c1.launch.py
```
### UBT
```
*setup commands as above*
1. ros2 launch my_robot_slam online_slam.launch.py
2. ros2 node list && ros2 topic list && ros2 topic echo /scan
```
## Full mapping test
### UBT
```
*setup commands as above*
3. ros2 bag record -s mcap -a -o test2
4. ros2 run rviz2 rviz2 -d ~/cartographer_ws/src/my_robot_slam/rviz/mapper.rviz
```
# Miscellaneous
## Updating RPI code
> scp .\PRODUCT\RPI\* foo@ood:~/Desktop/PRESENT/
> scp -r .\core foo@ood:~/robokedach_workspace; scp -r .\robot foo@ood:~/robokedach_workspace; scp -r .\config foo@ood:~/robokedach_workspace
## Connecting UBT and RPI to internet
1. Open "Network Connections"
2. Share the wifi connection. This will change the wsl_Bridge ip to 192.168.137.1
### UBT
```
sudo ip addr flush dev eth0
sudo ip addr add 192.168.137.0/24 dev eth0
sudo ip link set dev eth0 up
sudo ip route add default via 192.168.137.1 dev eth0
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null
```
### RPI
```
sudo ip addr flush dev eth0
sudo ip addr add 192.168.137.0/24 dev eth0
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
```