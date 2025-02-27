# FineNav2D

## How to use

1. clone the repository
   
```shell
mkdir nav_ws && cd nav_ws/
git clone https://gitee.com/huigg-practice/FineNav2D.git
cd FineNav2D/
git submodule update --init --recursive 
```

2. build the project

```shell
cd ..
colcon build --symlink-install
```

3. launch

```shell
# Make sure you are in the root directory of the project
source install/setup.bash
```
Example: Bring up FineNav2D with Livox Mid-360 and Fast-LIO

```shell
# Check your ethernet interface
ifconfig
# Set the static IP address of your ethernet interface
# The default ipv4 address is 192.168.1.50, you can change it in <PROJECT_DIR>/fn_bringup/config/MID360_config.yaml
sudo ifconfig <interface> <ip>
# Launch FineNav2D
ros2 launch fine_nav2d_bringup bringup_nav2_real.launch.py \
lidar_type:=livox \
enable_recorder:=false \
lio_type:=fast_lio \
enable_rviz:=true
```
If everything goes well, you will see the following result:
![expected_result.png](asset/expected_result.png)