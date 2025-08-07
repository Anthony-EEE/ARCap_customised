

```bash
susgrip_2f_driver/                      # ROS2 package
├── CMakeLists.txt                      # Build instructions (for C++ drivers)
├── package.xml                         # Package metadata
├── config/                             # Configuration files (YAML, JSON)
│   ├── params.yaml                     # ROS2 parameters for the driver
├── launch/                             # Launch files for starting the driver
│   ├── susgrip_2f_moveit.launch.py     # Launch script for ROS2
├── src/                                # Source code
│   ├── susgrip_2f_driver_node.py       # Main driver node
│   ├── susgrip_2f_driver_node.cpp      # Main driver node
├── include/                            # Header files (if using C++)
├── urdf/                               # URDF/Xacro files for visualization
├── msg/                                # Custom messages (if needed)
├── srv/                                # Custom services (if needed)
└── README.md                           # Documentation
```

