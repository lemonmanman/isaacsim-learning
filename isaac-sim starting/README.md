# Isaac-Sim Startup Quick Enter

This file helps me quick enter isaacsim.

## 1.Breakdown Preventing
This step is NECESSARY before the second STARTUP proscess.
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## 2.Startup
```bash
cd ~/isaacsim/_build/linux-x86_64/release
./isaac-sim.sh
```

## 3.Teleop Connecting
Open a new terminal and：
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
```

## 4.Robot Importing
- First check the type of file:
  - If: urdf --> import
  - If: usd --> open(directly)(double click is okay)
  - Choose YES when encountering messages below:
  ![pop up when opening the usd](../.images/message_when_opening_usd.png)
  > REASON: Some of the controlling scripts are personally written, so the scripts are necessary when the ros2 control is in need.

## Possible Errors
1. Once I encountered errors below, which follows with a quick shutdown 
when I attempt to open isaac sim:
```bash 
[6,730ms] [Error] [carb.scripting-python.plugin] RuntimeError: Unexpected error from cudaGetDeviceCount(). Did you run some cuda functions before calling NumCudaDevices() that might have already set an error? Error 804: forward compatibility was attempted on non supported HW
```
Furthermore, when will meet the situation below:
```bash
nvidia-smi

## you might see:
Failed to initialize NVML: Driver/library version mismatch
NVML library version: 580.126
```
This is because of the conflict between NVIDIA kernel driver version and NVIDIA Management Library (NVML) version.

You can fix this by simply rebooting your system:
```bash
sudo reboot
```