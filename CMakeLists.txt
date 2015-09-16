cmake_minimum_required(VERSION 2.8.3)
project(flirtlib_ros)

find_package(catkin REQUIRED COMPONENTS
   message_generation
   roscpp
   std_srvs
   sensor_msgs
   geometry_msgs
   std_msgs
   move_base_msgs
   actionlib_msgs
   actionlib
   tf
   occupancy_grid_utils
   interactive_markers
   warehouse_ros
   cmake_modules
   )

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(TinyXML REQUIRED)
find_package(FLIRT REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    ${FLIRT_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
    DescriptorRos.msg
    InterestPointRos.msg
    ScanMap.msg
    ExecutiveState.msg
    RefScanRos.msg
    Vector.msg)

add_action_files(
  DIRECTORY action
  FILES
    RotateInPlace.action)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_srvs
    sensor_msgs
    move_base_msgs
    actionlib_msgs
    actionlib
    tf
    occupancy_grid_utils
    interactive_markers
    warehouse_ros
  DEPENDS
    TinyXML
    FLIRT)

add_library(${PROJECT_NAME}
    src/conversions.cpp
    src/common.cpp
    src/flirtlib.cpp
    src/localization_monitor.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
  ${FLIRT_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(flirtlib_ros_test src/flirtlib_ros_test.cpp)
target_link_libraries(flirtlib_ros_test ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(markers src/interactive_markers.cpp)
target_link_libraries(markers ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

add_executable(simulate_scans src/simulate_scans.cpp)
target_link_libraries(simulate_scans ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

add_executable(place_rec_test src/place_rec_test.cpp)
target_link_libraries(place_rec_test ${PROJECT_NAME})

add_executable(generate_scan_map src/generate_scan_map.cpp)
target_link_libraries(generate_scan_map ${PROJECT_NAME})

add_executable(startup_loc src/startup_loc.cpp)
target_link_libraries(startup_loc ${PROJECT_NAME})

add_executable(localization_monitor src/localization_monitor_node.cpp)
target_link_libraries(localization_monitor ${PROJECT_NAME})

add_executable(generate_simulated_scans src/generate_simulated_scans.cpp)
target_link_libraries(generate_simulated_scans ${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(rotate_in_place src/rotate_in_place.cpp)
#target_link_libraries(rotate_in_place ${tinyxml_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME}
    flirtlib_ros_test
    markers
    simulate_scans
    place_rec_test
    generate_scan_map
    startup_loc
    localization_monitor
    generate_simulated_scans
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS
    scripts/wander.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    config
    launch
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
