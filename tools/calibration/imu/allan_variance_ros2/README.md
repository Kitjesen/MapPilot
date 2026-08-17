# Allan Variance ROS2

ROS2-port of this [ROS package](https://github.com/ori-drs/allan_variance_ros).

## ROS2 package which loads a rosbag of IMU data and computes Allan Variance parameters
The purpose of this tool is to read a long sequence of IMU data and compute the Angle Random Walk (ARW), Bias Instability and Gyro Random Walk for the gyroscope as well as Velocity Random Walk (VRW), Bias Instability and Accel Random Walk for the accelerometer.

While there are many open source tools which do the same thing, this package has the following features:

- Fully ROS2 compatible. Simply record a `MCAP` file and provide it as input. No conversion required.
- No need to play back the `MCAP` bag file.
- Designed for [Kalibr](https://github.com/ethz-asl/kalibr). Will produce an `imu.yaml` file.

## How to build

`colcon build --symlink-install`

## How to use

1. Place your IMU on some damped surface and record your IMU data to a rosbag. You must record **at least** 3 hours of data. The longer the sequence, the more accurate the results.

2. Run the Allan Variance computation tool (example config files provided):

  ``ros2 run allan_variance_ros2 allan_variance [path_to_folder_containing_bag] [path_to_config_file]``

3. This will compute the Allan Deviation for the IMU and generate a CSV. The next step is to visualize the plots and get parameters. For this run:

  ``python3 ./src/allan_variance_ros2/scripts/analysis.py --data allan_variance.csv``

  Press `space` to go to next figure.

## Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) is a useful collection of tools for calibrating cameras and IMUs. For IMU calibration it needs the noise parameters of the IMU generated in a yaml file. `allan_variance_ros` automatically generates this file file as `imu.yaml`:

```
#Accelerometer
accelerometer_noise_density: 0.006308226052016165 
accelerometer_random_walk: 0.00011673723527962174 

#Gyroscope
gyroscope_noise_density: 0.00015198973532354657 
gyroscope_random_walk: 2.664506559330434e-06 

rostopic: '/sensors/imu' #Make sure this is correct
update_rate: 400.0 #Make sure this is correct

```

## Original Author

[Russell Buchanan](https://www.ripl-lab.com/)

If you use this package, or the [original package](https://github.com/ori-drs/allan_variance_ros), for academic work, please consider using the citation below:

```
@software{AllanVarianceRos,
  author       = {Russell Buchanan},
  title        = {Allan Variance ROS},
  month        = Nov,
  year         = 2021,
  publisher    = {Oxford Robotics Institute, DRS Lab},
  version      = {1.2},
  url          = {https://github.com/ori-drs/allan_variance_ros}}
}
```

## ROS2-port Author:

[Christian-Nils Boda](https://github.com/christian-nils) ([Autoliv](https://www.autoliv.com/) Research)

## References

- [Indirect Kalman Filter for 3D Attitude Estimation, Trawny & Roumeliotis](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)
- [An introduction to inertial navigation, Oliver Woodman](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf) 
- [Characterization of Errors and Noises in MEMS Inertial Sensors Using Allan Variance Method, Leslie Barreda Pupo](https://upcommons.upc.edu/bitstream/handle/2117/103849/MScLeslieB.pdf?sequence=1&isAllowed=y)
- [Kalibr IMU Noise Documentation](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
