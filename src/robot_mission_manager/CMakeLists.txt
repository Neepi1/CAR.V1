cmake_minimum_required(VERSION 3.8)
project(robot_mission_manager)

find_package(ament_cmake REQUIRED)

add_library(mission_fsm
  src/mission_fsm.cpp
)
target_compile_features(mission_fsm PUBLIC cxx_std_17)
target_include_directories(mission_fsm
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mission_fsm
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_fsm test/test_mission_fsm.cpp)
  target_link_libraries(test_mission_fsm mission_fsm)
endif()

ament_export_include_directories(include)
ament_export_libraries(mission_fsm)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
