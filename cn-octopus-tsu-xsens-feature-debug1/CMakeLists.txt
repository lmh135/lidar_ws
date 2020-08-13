cmake_minimum_required(VERSION 3.5)
project(device_xsens)
set(CMAKE_VERBOSE_MAKEFILE true)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(middleware REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xsens_msgs REQUIRED)
find_package(transformation_util2 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src/application
  src/protocol
  src/stream
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(sensor_middle sensor_manager zmq zlog)

#############################################################

# build library
add_library(stream
  src/stream/serial.cc
)
target_link_libraries(stream -lboost_system)

add_library(xsens
  src/protocol/xsens.cc
  src/application/xsens_mt.cc
)
target_link_libraries(xsens -lboost_system stream middleware transformation_util2)
ament_target_dependencies(xsens Boost sensor_msgs xsens_msgs)

#############################################################

# build unit test TODO
# add_executable(test_stream src/unit_test/test_stream.cc)
# target_link_libraries(test_stream stream middleware)
# ament_target_dependencies(test_stream)
# 
# add_executable(test_protocol src/unit_test/test_protocol.cc)
# target_link_libraries(test_protocol stream novatel middleware)
# ament_target_dependencies(test_protocol)

#############################################################

# build zoro node
add_executable(xsens_node src/xsens_node.cc)
target_link_libraries(xsens_node stream xsens middleware)
ament_target_dependencies(xsens_node)

#############################################################

install(DIRECTORY launch/ DESTINATION launch/${PROJECT_NAME})
install(TARGETS stream DESTINATION lib/${PROJECT_NAME})
install(TARGETS xsens DESTINATION lib/${PROJECT_NAME})
install(TARGETS xsens_node DESTINATION lib/${PROJECT_NAME})
#install(TARGETS test_stream DESTINATION lib/${PROJECT_NAME})
#install(TARGETS test_protocol DESTINATION lib/${PROJECT_NAME})
#install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

ament_package()
