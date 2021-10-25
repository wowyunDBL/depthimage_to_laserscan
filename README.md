### depthimage_to_laserscan
=======================
It is modified from ros_package.

Using height layer and grid filter mask. 
### file flow
```dot
digraph G {
    depthimage_to_laserscan.cpp -> DepthImageToLaserScanROS
}
```

```
depthimage_to_laserscan.cpp # initiate DepthImageToLaserScanROS dtl 
```

Converts a depth image to a laser scan for use with navigation and localization.

ROS Wiki Page:
http://www.ros.org/wiki/depthimage_to_laserscan
