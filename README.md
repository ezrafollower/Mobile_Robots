# Mobile_Robots
EE622  => 192.168.50.195

EE627  => 192.168.50.231

EE632  => 192.168.50.195

Jetson nano compile
```bash
    cd nano_ws
    git clone -b ${ROS_DISTRO} https://github.com/ros-perception/vision_opencv.git src/vision_opencv
    catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/$(arch)-linux-gnu/libpython3.6m.so
```
