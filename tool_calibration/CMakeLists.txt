cmake_minimum_required(VERSION 3.0.2)
project(tool_calibration)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf_conversions
  sss_msgs
  message_generation
)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB EXTRA_FILES scripts/*.py test/*.py src/*/*.py plans/*.xml demo/*.xml launch/*.launch action/* srv/* msg/* config/*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
   FILES
   CalibrationUrdfUpdate.srv
   CalibrationUrdfRetrieve.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp
  rospy
  tf
  tf_conversions
  sss_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
  src/tool_point_calibration.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(tool_calibration_server
 src/tool_calibration_server.cpp
 )

add_dependencies(tool_calibration_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(tool_calibration_server
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  tool_calibration
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tool_point_calibration tool_point_calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
