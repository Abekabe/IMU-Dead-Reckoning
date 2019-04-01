# SDC Homework -- IMU Integrator

Given ROS bag with IMU data, integrator the vehicle or human path. 

## Getting Started

### Prerequisites
You need to prepare ROS system, then Eigen3 is also requested which can be installed like:
```
sudo apt-get install libeigen3-dev
```

### Installing

You can download the code from github directly or use command line:

```
git pull https://github.com/Abekabe/verdict-crawler.git
cd verdict-crawler-master
catkin_make
```


## Running the tests
First, you need to run the roscore, play the rosbag with IMU data, then execute the node:
```
roscore
rosbag play [your bag]
rosrun hw3_0410817 hw3_node
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
