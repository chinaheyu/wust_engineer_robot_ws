# wust_engineer_robot_ws
ROS code for RoboMaster University Technical Challenge 

## 使用指南
### 1. 配置STM32的USB虚拟串口映射规则

首先连接STM32设备的虚拟串口，lsusb可以查看Vendor和Product的ID，然后创建并配置/etc/udev/rules.d/roborts.rules文件

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", SYMLINK+="serial_sdk"
```
重启udev服务
```shell
sudo service udev reload
sudo service udev restart
```
可能还需要重新插拔设备

### 2. 编译librealsense
```shell
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=false -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/home/dji/miniforge3/bin/python3.8
make -j$(($(nproc)-1))
sudo make install
```

