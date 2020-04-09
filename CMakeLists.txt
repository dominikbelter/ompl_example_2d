cmake_minimum_required(VERSION 2.8.3)
project(ompl_example_2d)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    eigen_conversions
#    moveit_ros_planning_interface
#    moveit_ros_move_group
    std_srvs
    message_generation
    ompl
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

find_package(OMPL)

find_package(Boost REQUIRED COMPONENTS system)
#  add_service_files(
#    FILES
#    RG2.srv
# )
generate_messages(DEPENDENCIES std_msgs sensor_msgs) 
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}_library
    CATKIN_DEPENDS
	    roscpp
	    message_filters
            sensor_msgs
	    std_msgs
            geometry_msgs
	    tf
	    tf_conversions
	    eigen_conversions
#            moveit_ros_planning_interface
#            moveit_ros_move_group
	    trajectory_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
file(GLOB ompl_example_2d_sources
    src/ompl_example_2d.cpp
)

file(GLOB ompl_example_2d_headers
    include/ompl_example_2d/*.hpp
)

add_library(${PROJECT_NAME}_library STATIC ${ompl_example_2d_sources} ${ompl_example_2d_headers}
)

target_link_libraries(${PROJECT_NAME}_library
   ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)
 
# add_dependencies(${PROJECT_NAME}_library ompl_example_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


## Declare a cpp executable
add_executable(
  ${PROJECT_NAME}_node
  src/ompl_example_2d_node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME} ompl_example_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

## Declare a cpp executable
# add_executable(
#   ${PROJECT_NAME}_trajectory_tending
#   src/trajectoryNodeTending.cpp
# )

# add_dependencies(${PROJECT_NAME}_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ompl_example_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_elevation_mapping.cpp
#  test/ElevationMapTest.cpp
#  test/WeightedEmpiricalCumulativeDistributionFunctionTest.cpp
#)
#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
#endif()
