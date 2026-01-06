```bash
sudo apt install python3-dev python3-pip
sudo pip3 install websocket-client
```

Remember change the ACCID in the code to your robot's serial number (SN)

```bash
python3 odom_publisher.py
```

/limx/wheel_odom is the odometry topic

/limx/imu is the imu topic
