cmake_minimum_required(VERSION 3.16)
project(hydrofoil_physics)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

# ---- pure force-model library (no Gazebo dependency)
add_library(foil STATIC src/foil_model.cpp)
target_include_directories(foil PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(foil PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- Gazebo system plugin
add_library(FoilDynamics SHARED src/FoilDynamics.cc)
target_link_libraries(FoilDynamics PRIVATE foil gz-sim::gz-sim gz-plugin::gz-plugin gz-transport::gz-transport)

install(TARGETS FoilDynamics LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

# ---- golden-value test against the Python model
if(BUILD_TESTING)
  add_executable(test_foil_model test/test_foil_model.cpp)
  target_link_libraries(test_foil_model foil)
  target_include_directories(test_foil_model PRIVATE test)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_foil_model COMMAND $<TARGET_FILE:test_foil_model>)
endif()

ament_package()
