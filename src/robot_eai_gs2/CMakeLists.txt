cmake_minimum_required(VERSION 3.8)
project(robot_eai_gs2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(GS2_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/EaiSdkForGS2)

file(GLOB GS2_SDK_SOURCES CONFIGURE_DEPENDS
  ${GS2_SDK_ROOT}/src/*.cpp
  ${GS2_SDK_ROOT}/src/*.c
  ${GS2_SDK_ROOT}/src/*.h
  ${GS2_SDK_ROOT}/src/impl/unix/*.cpp
  ${GS2_SDK_ROOT}/src/impl/unix/*.h
)

add_library(eai_gs2_sdk STATIC ${GS2_SDK_SOURCES})
target_include_directories(eai_gs2_sdk PUBLIC
  ${GS2_SDK_ROOT}/include
  ${GS2_SDK_ROOT}/src
)
target_compile_features(eai_gs2_sdk PUBLIC cxx_std_11)
target_link_libraries(eai_gs2_sdk PUBLIC pthread rt)

add_executable(gs2_driver_node src/gs2_driver_node.cpp)
target_compile_features(gs2_driver_node PUBLIC cxx_std_17)
ament_target_dependencies(gs2_driver_node geometry_msgs rclcpp sensor_msgs std_srvs)
target_link_libraries(gs2_driver_node eai_gs2_sdk)

install(TARGETS
  gs2_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  third_party/EaiSdkForGS2/LICENSE.BSD
  DESTINATION share/${PROJECT_NAME}/licenses
)

ament_package()
