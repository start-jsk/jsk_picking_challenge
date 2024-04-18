cmake_minimum_required(VERSION 2.8.3)
project(jsk_arc2017_baxter)

# force_proximity_ros can not be found on build.ros.org
find_package(force_proximity_ros QUIET)
if(force_proximity_ros_FOUND)
  set(FORCE_PROXIMITY_ROS_PKG force_proximity_ros)
else()
  message(WARNING "COULD NOT BE FOUND force_proximity_ros PACKAGE")
endif()

find_package(catkin REQUIRED COMPONENTS
    ${FORCE_PROXIMITY_ROS_PKG}
    message_generation
    roseus
    std_msgs
    baxter_core_msgs
    hardware_interface
    transmission_interface
    controller_manager
    dynamixel_controllers
    dynamixel_msgs
    actionlib_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES
    GripperSensorStates.msg
)

add_service_files(
    FILES
    UpdateState.srv
    GetState.srv
    CheckCanStart.srv
)

add_action_files(
    DIRECTORY action
    FILES CalibJoint.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    ${FORCE_PROXIMITY_ROS_PKG}
    actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# XXX

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}_transmission_loader_plugins
    CATKIN_DEPENDS
    message_runtime
    ${FORCE_PROXIMITY_ROS_PKG}
    std_msgs
    baxter_core_msgs
    hardware_interface
    transmission_interface
    controller_manager
    dynamixel_msgs
    actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## -------------------
## baxter.l generation
## -------------------
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/baxter.l
  COMMAND rosrun euscollada collada2eus baxter.dae baxter.yaml baxter.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxter.dae ${PROJECT_SOURCE_DIR}/robots/baxter.yaml)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/baxter.dae
  COMMAND rosrun collada_urdf urdf_to_collada baxter.urdf baxter.dae
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxter.urdf)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/baxter.urdf
  COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} rosrun xacro xacro baxter.xacro > baxter.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxter.xacro
          ${PROJECT_SOURCE_DIR}/robots/left_vacuum_gripper.xacro
          ${PROJECT_SOURCE_DIR}/robots/right_vacuum_gripper.xacro)
## -------------------
## baxterlgv7.l generation
## -------------------
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.l
  COMMAND rosrun euscollada collada2eus baxterlgv7.dae baxterlgv7.yaml baxterlgv7.l
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.dae ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.yaml)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.dae
  COMMAND rosrun collada_urdf urdf_to_collada baxterlgv7.urdf baxterlgv7.dae
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.urdf)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.urdf
  COMMAND ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH} rosrun xacro xacro baxterlgv7.xacro > baxterlgv7.urdf
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/robots
  DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.xacro
          ${PROJECT_SOURCE_DIR}/robots/left_gripper_v7.xacro
          ${PROJECT_SOURCE_DIR}/robots/right_vacuum_gripper.xacro)
# get collada-dom version
find_package(PkgConfig)
pkg_check_modules(COLLADA collada-dom>=2.4.4)
if(${COLLADA_FOUND})
  add_custom_target(generate_baxter_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxter.l)
  add_custom_target(generate_baxterlgv7_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/robots/baxterlgv7.l)
else()
  pkg_check_modules(COLLADA collada-dom)
  message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
endif()

# Transmission loader library
add_library(${PROJECT_NAME}_transmission_loader_plugins
  src/tendon_transmission_loader.cpp include/jsk_arc2017_baxter/tendon_transmission_loader.h)
target_link_libraries(${PROJECT_NAME}_transmission_loader_plugins ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(gripper_v6_ros_control_node src/gripper_v6_ros_control.cpp)

## Add cmake target dependencies of the executable
add_dependencies(gripper_v6_ros_control_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gripper_v6_ros_control_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY
  config euslisp launch meshes robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  gripper_v6_ros_control_node ${PROJECT_NAME}_transmission_loader_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python(node_scripts)
  roslint_add_test()
  find_package(rostest REQUIRED)
  add_rostest(test/test_baxter.test)
  add_rostest(test/test_baxterlgv7.test)
  add_rostest(test/test_baxter_interface.test)
  add_rostest(test/test_baxterlgv7_interface.test)
  add_rostest(test/test_pick_state_server.test)
  add_rostest(test/test_stow_state_server.test)
  add_rostest(test/test_pick_main.test)
  add_rostest(test/test_stow_main.test)
  add_rostest(test/test_arc_interface.test)
  add_rostest(test/test_stow_interface.test)
  find_package(jsk_tools REQUIRED)
  find_package(jsk_apc2016_common REQUIRED)
  if(${jsk_tools_VERSION} VERSION_GREATER 2.0.13)
    jsk_tools_add_shell_test(COMMAND "rosrun jsk_apc2016_common euslint ${PROJECT_SOURCE_DIR}/euslisp")
    jsk_tools_add_shell_test(COMMAND "rosrun jsk_apc2016_common euslint ${PROJECT_SOURCE_DIR}/test")
  endif()
  find_package(jsk_arc2017_common REQUIRED)
  add_rostest(test/test_pick_interface.test)
endif()
