cmake_minimum_required(VERSION 3.16)
project(njrh_fastdds_regression LANGUAGES CXX)

# Compile the official release's unmodified flow-controller tests, not a
# substitute test that merely starts a ROS process successfully.
if(NOT EXISTS "${FASTDDS_SOURCE}/test/blackbox/common/BlackboxTestsPubSubFlowControllers.cpp")
  message(FATAL_ERROR "FASTDDS_SOURCE must point to the official 2.6.12 source")
endif()
find_package(fastrtps 2.6 REQUIRED)
find_package(fastcdr REQUIRED)
find_package(foonathan_memory REQUIRED)
find_package(GTest REQUIRED)
set(BLACKBOX "${FASTDDS_SOURCE}/test/blackbox")
set(types HelloWorld KeyedHelloWorld StringTest Data64kb Data1mb
  KeyedData1mb FixedSized UnboundedHelloWorld)
set(type_sources "${BLACKBOX}/types/HelloWorldTypeObject.cxx")
foreach(type IN LISTS types)
  list(APPEND type_sources "${BLACKBOX}/types/${type}.cxx"
    "${BLACKBOX}/types/${type}PubSubTypes.cxx")
endforeach()
add_executable(flow_controller_regression
  test_main.cpp
  "${BLACKBOX}/common/BlackboxTestsPubSubFlowControllers.cpp"
  ${type_sources}
  "${BLACKBOX}/utils/data_generators.cpp"
  "${BLACKBOX}/utils/lambda_functions.cpp"
  "${BLACKBOX}/utils/print_functions.cpp")
target_compile_features(flow_controller_regression PRIVATE cxx_std_14)
target_compile_definitions(flow_controller_regression PRIVATE
  ASIO_STANDALONE BOOST_ASIO_STANDALONE)
target_include_directories(flow_controller_regression PRIVATE
  "${BLACKBOX}/common" "${BLACKBOX}/api/dds-pim")
target_link_libraries(flow_controller_regression PRIVATE
  fastrtps fastcdr foonathan_memory GTest::gtest pthread)
