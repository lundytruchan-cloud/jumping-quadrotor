cmake_minimum_required(VERSION 3.8)
project(quadcopter_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(spring_damper_leg_system SHARED
  src/spring_damper_leg_system.cc)
target_include_directories(spring_damper_leg_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(spring_damper_leg_system
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
install(TARGETS spring_damper_leg_system DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_spring_force test/test_spring_force.cpp)
  if(TARGET test_spring_force)
    target_include_directories(test_spring_force PRIVATE include)
    target_link_libraries(test_spring_force spring_damper_leg_system)
  endif()
endif()

ament_package()
