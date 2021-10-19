# Occupancy Mapping Release Notes

## [v0.1]

2021-10-19

- Different height extraction 
- Mapping 3D point cloud into 2D range
- Transform to rosmsg type for mapping usage

## file tree
```
├── include
│   └── depthimage_to_laserscan
│       ├── DepthImageToLaserScan.h
│       ├── DepthImageToLaserScan_raw.h   # origin file
│       ├── DepthImageToLaserScanROS.h  
│       ├── DepthImageToLaserScan_superSlow.h # eigen matrix file
│       └── depth_traits.h
├── src
│   ├── depthimage_to_laserscan.cpp
│   ├── DepthImageToLaserScan.cpp
│   ├── DepthImageToLaserScanNodelet.cpp
│   └── DepthImageToLaserScanROS.cpp

```

## Template for commit mdg
What/How
[New]
[Fix]
[Modi]