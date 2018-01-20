cmake_minimum_required(VERSION 2.8.3)
project(api)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS std_msgs geometry_msgs rospy)
set(SYS_DEPS python-serial python-sys)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Install python scripts using distutils
catkin_python_setup()

catkin_package(
    DEPENDS ${SYS_DEPS}
    CATKIN_DEPENDS ${CATKIN_DEPS}
)

install(PROGRAMS src/node
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
