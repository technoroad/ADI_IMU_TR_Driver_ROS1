cmake_minimum_required(VERSION 2.8.3)
project(adi_imu_tr_driver_ros1)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  std_srvs
  tf
  message_generation
  )

add_service_files(
  FILES
  SimpleCmd.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
#  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp sensor_msgs std_srvs
  )

include_directories(
  lib/include
  ${catkin_INCLUDE_DIRS}
  )


# Added by Technoroad
add_subdirectory(lib)

add_executable(adis_rcv_csv_node src/adis_rcv_csv_node.cpp)
target_link_libraries(adis_rcv_csv_node ${catkin_LIBRARIES} adis_rcv_csv)

install(TARGETS adis_rcv_csv_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )


