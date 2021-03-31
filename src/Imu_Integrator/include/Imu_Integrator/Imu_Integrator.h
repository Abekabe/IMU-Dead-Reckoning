#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/Marker.h>
#include<sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <cmath>
// Custom message includes. Auto-generated from msg/ directory.
/*
struct Orientation
{

};

struct Position
{
    double x, y, z;
};
*/
struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Matrix3d orien;
};

class ImuIntegrator
{
private:
    Pose pose;
    ros::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    visualization_msgs::Marker path;
    ros::Publisher line_pub;
    double deltaT;
    bool firstT;
public:
    //! Constructor.
    ImuIntegrator(const ros::Publisher&);
    //! Destructor.
    ~ImuIntegrator();

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage();

    //! Callback function for subscriber.
    void ImuCallback(const sensor_msgs::Imu &msg);

    void setGravity(const geometry_msgs::Vector3 &msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::Vector3 &msg);
    void calcOrientation(const geometry_msgs::Vector3 &msg);
};

#endif // SR_NODE_EXAMPLE_CORE_H
