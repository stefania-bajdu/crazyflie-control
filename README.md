# crazyflie-control
ROS2 packages for controlling a swarm of crazyflie nanodrones.

## NatNet SDK Setup (Required for Motion Capture)

This project requires the [NatNet SDK](https://optitrack.com/support/downloads/developer-tools.html#natnet-sdk) to interface with the motion capture system.

### 🔧 Installation Instructions

1. Download the **NatNet SDK** from the OptiTrack developer page: https://optitrack.com/support/downloads/developer-tools.html#natnet-sdk

2. Extract the downloaded archive.

3. Copy the shared library `libNatNet.so` to the following path in your workspace:
```bash 
ros_ws/src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/NatNetSDKCrossplatform/lib/ubuntu/libNatNet.so
```
or in the OS's user directory directly:
```bash 
/usr/local/lib/libNatNet.so
```


4. Export in the Ubuntu terminal :
```bash
export LD_LIBRARY_PATH=path_to_ros_ws/src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/NatNetSDKCrossplatform/lib:$LD_LIBRARY_PATH
```
or
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

