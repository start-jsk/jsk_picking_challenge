cmake_minimum_required(VERSION 2.8.3)
project(jsk_apc2016_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    message_generation
    std_msgs
    jsk_data
    jsk_gui_msgs
    jsk_recognition_msgs
    sensor_msgs
    message_filters
    tf2
    tf2_ros
    tf2_eigen
    pcl_ros
    cmake_modules
)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  BinInfo.msg
  BinInfoArray.msg
  SegmentationInBinSync.msg
  )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UpdateTarget.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  jsk_gui_msgs # jsk_2016_01_baxter_apc/euslisp/jsk_2016_01_baxter_apc/baxter-interface.l and jsk_arc2017_baxter/euslisp/lib/arc-interface.l
  jsk_recognition_msgs
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
  jsk_recognition_msgs
  message_filters
  sensor_msgs
  DEPENDS
  Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRECTORIES}
  )

## Declare a C++ library
# add_library(jsk_apc2016_common
#   src/${PROJECT_NAME}/jsk_apc2016_common.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(jsk_apc2016_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
# add_executable(jsk_apc2016_common_node src/jsk_apc2016_common_node.cpp)
add_executable(sib_topic_synchronizer src/sib_topic_synchronizer.cpp)
add_executable(cloud_to_spatial_features src/cloud_to_spatial_features.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(jsk_apc2016_common_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sib_topic_synchronizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cloud_to_spatial_features ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(jsk_apc2016_common_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(sib_topic_synchronizer ${catkin_LIBRARIES})
target_link_libraries(cloud_to_spatial_features ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

# trained data
# add_custom_target(install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)
# sample data
# add_custom_target(install_sample_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)

#############
## Install ##
#############

# install to global bin
set(BIN_EXECUTABLES euslint)
foreach(exec ${BIN_EXECUTABLES})
  add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/${exec}
    COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}
    COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/scripts/${exec} ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/
    DEPENDS ${PROJECT_SOURCE_DIR}/scripts/${exec})
  add_custom_target(install_${exec} ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/${exec})
  install(PROGRAMS scripts/${exec} DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endforeach(exec)

## Mark executables and/or libraries for installation
install(TARGETS sib_topic_synchronizer cloud_to_spatial_features
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/sib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## rqt plugin
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if($ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
  install(DIRECTORY node_scripts
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          USE_SOURCE_PERMISSIONS
  )
  install(DIRECTORY scripts
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          USE_SOURCE_PERMISSIONS
  )
else()
  install(DIRECTORY scripts
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          USE_SOURCE_PERMISSIONS
          PATTERN "*"
          PATTERN "*/*.py" EXCLUDE
  )
  install(DIRECTORY node_scripts
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          USE_SOURCE_PERMISSIONS
          PATTERN "*"
          PATTERN "*/*.py" EXCLUDE
  )

  file(GLOB SCRIPT_PROGRAMS scripts/*.py)
  catkin_install_python(
    PROGRAMS ${SCRIPT_PROGRAMS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  )
  file(GLOB SCRIPT_PROGRAMS node_scripts/*.py)
  catkin_install_python(
    PROGRAMS ${SCRIPT_PROGRAMS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
  )
endif()

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jsk_apc2016_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  # lint python code
  file(GLOB_RECURSE _py_files *.py)
  file(GLOB_RECURSE _exclude_files python/jsk_apc2016_common/rbo_segmentation/*.py)
  if(_exclude_files)  # condition for not initialized rbo_segmentation submodule
    list(REMOVE_ITEM _py_files ${_exclude_files})
  endif()
  roslint_python(${_py_files})
  roslint_add_test()
  # unit tests
  catkin_add_nosetests(python/jsk_apc2016_common/tests)
  find_package(rostest REQUIRED)
  add_rostest(test/publish_bin_bbox.test)
  find_package(jsk_tools REQUIRED)
  if(${jsk_tools_VERSION} VERSION_GREATER 2.0.13)
    jsk_tools_add_shell_test(COMMAND "rosrun jsk_apc2016_common euslint ${PROJECT_SOURCE_DIR}/samples")
  endif()
endif()
