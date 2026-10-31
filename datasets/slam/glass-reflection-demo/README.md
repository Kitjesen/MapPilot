# Reflection_detection

# Paper 

The paper describing the method was accepted for publication at XXX. 
Zhao, X.; Yang, Z.; Schwertfeger, S. Mapping with Reflection - Detection and Utilization of Reflection in 3D Lidar Scans. 
In Proceedings of the 2020 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR), 2020, pp. 27–33.
```
@INPROCEEDINGS{9292595,
  author={Zhao, Xiting and Yang, Zhijie and Schwertfeger, Sören},
  booktitle={2020 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)}, 
  title={Mapping with Reflection - Detection and Utilization of Reflection in 3D Lidar Scans}, 
  year={2020},
  volume={},
  number={},
  pages={27-33},
  keywords={Glass;Laser radar;Windows;Three-dimensional displays;Robots;Laser beams;Simultaneous localization and mapping},
  doi={10.1109/SSRR50563.2020.9292595}
  }
```
# Installation

Download the code to your ros workspace 
Use catkin_make to compile the code

# Datasets

Download link: https://3dref.github.io/

# How to run
Single frame algorithm for Velodyne HDL-32E

```
    roslaunch reflection_detection single_frame_detection_velodyne.launch
    rosbag play /demo_data/dual_pointcloud.bag
```

Single frame algorithm for 3DRef Hesai Pandar QT64
Need to change data place in line 86-87 in single_frame_detection_3dref.launch

```
    roslaunch reflection_detection single_frame_detection_3dref.launch
```

Multiple frame algorithm for 3DRef Hesai Pandar QT64
Need to change data place in line 86-87 in single_frame_detection_3dref.launch

```
    roslaunch reflection_detection multi_frame_detection_3dref.launch
```