cmake_minimum_required(VERSION 3.0.2)
project(ed_python)

find_package(catkin REQUIRED
  COMPONENTS
  ed_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS ed_msgs rospy
  DEPENDS
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
