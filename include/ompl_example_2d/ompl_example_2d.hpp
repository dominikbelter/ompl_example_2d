/*
 * ompl_example_2d.hpp
 *
 *  Created on: April 6, 2020
 *  Updated on: April 8, 2023
 *      Author: Dominik Belter
 *	 Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#pragma once

// ROS
#include <geometry_msgs/msg/pose_stamped.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// #include <moveit/ompl_interface/ompl_interface.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>

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
   */
    Planner2D(void);

    /*!
   * Destructor.
   */
    virtual ~Planner2D();

    /*!
   * plan path
   */
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& globalMap);

private:

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
    nav_msgs::msg::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
