/*
 * ompl_example_2d_node.cpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include <ros/ros.h>
#include "../include/ompl_example_2d/ompl_example_2d.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>

nav_msgs::OccupancyGrid globalMap;

// occupancy map callback
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg) {
    globalMap = *mapMsg;
}


int main(int argc, char** argv)
{
    // init ROS node
    ros::init(argc, argv, "ompl_example_2d");

    // create node handler
    ros::NodeHandle nodeHandle("~");
    ompl_example_2d::Planner2D planner_(nodeHandle);

    // setup the ROS loop rate
    ros::Rate loop_rate(1);

    // planned path publisher
    ros::Publisher path_pub = nodeHandle.advertise<nav_msgs::Path>("planned_path", 1000);

    // occupancy map subscriber
    ros::Subscriber map_sub = nodeHandle.subscribe("/map", 10, mapCallback);

    while (ros::ok()){
        nav_msgs::Path plannedPath;
        plannedPath = planner_.planPath(globalMap);

        // publish the planned path
        path_pub.publish(plannedPath);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
