# Generate robot description file for function input file of RobotArrayRvizPlugins
Reference for robot description: 
- [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- [franka_description](https://github.com/frankaemika/franka_description)

Run scripts to generate the URDF files from the xacro files:
```bash
python3 create_urdf.py
```

The two file: fr3.urdf, ur5.urdf