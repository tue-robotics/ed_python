cmake_minimum_required(VERSION 3.0.2)
project(ed_python)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_python_setup()

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)

  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore DESCRIPTION_BOILERPLATE")
endif()
