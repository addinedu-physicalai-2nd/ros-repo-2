cmake_minimum_required(VERSION 3.8)
project(fleet_graph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fleet_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# yaml-cpp 0.8(Ubuntu 24.04)은 yaml-cpp::yaml-cpp, 0.7은 yaml-cpp
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

# ---- 라이브러리: 메인 서비스가 링크해서 쓴다 ----
add_library(${PROJECT_NAME} src/graph.cpp src/graph_loader.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_TARGET})

# ---- 실행 파일 ----
add_executable(graph_check src/graph_check.cpp)
target_link_libraries(graph_check ${PROJECT_NAME})

add_executable(example_server src/example_server.cpp)
target_link_libraries(example_server ${PROJECT_NAME} rclcpp::rclcpp ament_index_cpp::ament_index_cpp ${fleet_msgs_TARGETS})

# ---- 설치 ----
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS graph_check example_server DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)
ament_package()
