/*
 * ompl_example_2d.hpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *	 Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace ompl_example_2d {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:

    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
    Planner2D(ros::NodeHandle& _nodeHandle);

    /*!
   * Destructor.
   */
    virtual ~Planner2D();

    /*!
   * plan path
   */
    nav_msgs::Path planPath(const nav_msgs::OccupancyGrid& globalMap);

private:
    /// node handle
    ros::NodeHandle& nodeHandle;

    /// problem dim
    int dim;

    /// max step length
    double maxStepLength;

    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// start position
    std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
    std::shared_ptr<ompl::base::StateSpace> space;

    /// configure node
    void configure(void);

    /// extract path
    nav_msgs::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
