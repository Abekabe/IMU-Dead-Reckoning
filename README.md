# IMU Dead Reckoning

Given ROS bag with IMU data, integrator the vehicle or human path.
Here is a demo video:

[![Demo Video](https://drive.google.com/file/d/1r0dGFB5iiyyESFEeKqCsn3dEpLfnaSJd/view?usp=sharing)](https://drive.google.com/file/d/1Z4PAO3LoPApbf5XNheIb-J3KEl13hMEx/view?usp=sharing)

## Getting Started

### Prerequisites
You need to prepare ROS system, then Eigen3 is also requested which can be installed like:
```
sudo apt-get install libeigen3-dev
```

### Installing

You can download the code from github directly or use command line:

```
git pull https://github.com/Abekabe/IMU-Dead-Reckoning.git
cd IMU-Dead-Reckoning
catkin_make
```


## Running the tests
First, you need to run the roscore, play the rosbag with IMU data, then execute the node:
```
roscore
rosbag play [your bag]
rosrun Imu_Integrator Imu_Integrator_node
```


Notice that the IMU topic is **/imu/data** and the marker which been published is in the **/globa** frame


## Contributing

1. Fork it (https://github.com/yourname/yourproject/fork)
2. Create your feature branch (git checkout -b feature/fooBar)
3. Commit your changes (git commit -am 'Add some fooBar')
4. Push to the branch (git push origin feature/fooBar)
5. Create a new Pull Request

## Versioning

* 0.1.0
    + The first proper release

## Authors

* **Caleb Lee** - *Initial work* - [Abekabe](https://github.com/Abekabe)

## License

This project is licensed under the GPL License.
