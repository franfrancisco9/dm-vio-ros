cmake_minimum_required(VERSION 3.5)
project(dmvio_ros2)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer regex REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Pangolin 0.2 REQUIRED)
find_package(OpenCV REQUIRED)

set(DMVIO_BUILD $ENV{DMVIO_BUILD})
message("\n---- Path to DM-VIO build folder is: ${DMVIO_BUILD}\n")

set(DMVIO_ROOT ${DMVIO_BUILD}/..)
find_library(DMVIO_LIBRARY dmvio ${DMVIO_BUILD}/lib/)

if(${Boost_VERSION} GREATER_EQUAL 106500)
  message("Building with stacktrace support.")
  set(STACKTRACE_LIBRARIES dl)
  set(STACKTRACE_DEFINES "-DSTACKTRACE -DBOOST_STACKTRACE_USE_ADDR2LINE")
endif()

add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS}"
)
set(CMAKE_CXX_FLAGS_DEBUG
    "-O2 -g -fno-omit-frame-pointer -DEIGEN_INITIALIZE_MATRICES_WITH_NAN -DDEBUG ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "-O3 -g -fno-omit-frame-pointer ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELEASE
    "-O3 -DNDEBUG"
)

# Message generation
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DMVIOPoseMsg.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_export_dependencies(geometry_msgs sensor_msgs cv_bridge rclcpp rosbag2_cpp rosbag2_storage_default_plugins)

include_directories(
  include
  ${DMVIO_ROOT}/src
  ${DMVIO_ROOT}/src/dso
  ${EIGEN3_INCLUDE_DIR}
  ${DMVIO_ROOT}/thirdparty/Sophus
  ${ament_cmake_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/ROSOutputWrapper.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp cv_bridge geometry_msgs sensor_msgs rosbag2_cpp rosbag2_storage_default_plugins)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${DMVIO_LIBRARY}
  ${DMVIO_LINKED_LIBRARIES}
  ${ament_cmake_LIBRARIES}
  ${cpp_typesupport_target}
  gtsam
  pangolin
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
